using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Globalization;
using System.IO;
using System.IO.Ports;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading;
using System.Web.Script.Serialization;
using System.Windows.Forms;

internal static class F405ClawCalibrator
{
    [STAThread]
    private static int Main(string[] args)
    {
        Application.EnableVisualStyles();
        Application.SetCompatibleTextRenderingDefault(false);
        try
        {
            if (args.Length != 4) throw new ArgumentException("Use tools/Start-F405ClawCalibrator.ps1.");
            Application.Run(new CalibratorForm(args[0], args[1], args[2], args[3]));
            return 0;
        }
        catch (Exception error) { MessageBox.Show(error.Message, "F405夹爪微调"); return 1; }
    }
}

internal sealed class CalibratorForm : Form
{
    private readonly string repo, openOcdExe, dataDirectory;
    private readonly bool inverted;
    private readonly TextBox port = new TextBox();
    private readonly NumericUpDown opening = new NumericUpDown();
    private readonly NumericUpDown leftOffset = new NumericUpDown();
    private readonly NumericUpDown rightOffset = new NumericUpDown();
    private readonly ComboBox step = new ComboBox();
    private readonly Label preview = new Label();
    private readonly Label state = new Label();
    private readonly TextBox history = new TextBox();
    private readonly CheckBox referenceConfirmed = new CheckBox();
    private readonly JavaScriptSerializer json = new JavaScriptSerializer();
    private readonly StreamWriter session;
    private readonly object helperLogLock = new object();
    private readonly StringBuilder helperLog = new StringBuilder();
    private SerialPort serial;
    private Process helper;
    private TcpClient rpcClient;
    private NetworkStream rpcStream;
    private Dictionary<string, object> saved;
    private string consoleIdentity = "";
    private bool motionService, connected, loading, hasRead;
    private int liveLeft, liveRight;

    public CalibratorForm(string repository, string openOcd, string directory, string portName)
    {
        repo = Path.GetFullPath(repository);
        openOcdExe = Path.GetFullPath(openOcd);
        dataDirectory = Path.GetFullPath(directory);
        Directory.CreateDirectory(dataDirectory);
        inverted = Regex.IsMatch(File.ReadAllText(Path.Combine(repo, "Main/Inc/app_config.h")),
            @"(?m)^\s*#define\s+APP_CLAW_SERVOS_INVERTED\s+1\b");
        session = new StreamWriter(Path.Combine(dataDirectory,
            "adjust_" + DateTime.Now.ToString("yyyyMMdd_HHmmss_fff") + ".jsonl"), true, new UTF8Encoding(false));
        session.AutoFlush = true;
        Text = "F405 双爪微调 · 1 µs分度";
        Font = new Font("Microsoft YaHei UI", 10F);
        Size = new Size(980, 750);
        MinimumSize = new Size(850, 650);
        StartPosition = FormStartPosition.CenterScreen;
        AutoScaleMode = AutoScaleMode.Dpi;

        var layout = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 1, RowCount = 10, Padding = new Padding(12) };
        for (int row = 0; row < 9; ++row) layout.RowStyles.Add(new RowStyle(SizeType.AutoSize));
        layout.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
        Controls.Add(layout);
        layout.Controls.Add(Note("只调双爪，保留原S命令；不烧录、不发MOVE/TURN、不改升降或相机。关闭窗口保持最后脉宽。"), 0, 0);
        var connection = Row();
        connection.Controls.Add(new Label { Text = "程序器串口", AutoSize = true, Margin = new Padding(3, 10, 3, 3) });
        port.Text = portName; port.Width = 90; connection.Controls.Add(port);
        connection.Controls.Add(Button("连接并读取", delegate { ConnectBoard(); }));
        connection.Controls.Add(Button("断开并保持", delegate { DisconnectBoard(); Log("已断开，保持最后脉宽。"); }));
        connection.Controls.Add(Button("恢复旧合爪 S4=86 / S2=90", delegate { RestoreOldClosed(); }));
        layout.Controls.Add(connection, 0, 1);

        state.AutoSize = true; state.Text = "未连接；打开窗口不会自动移动。"; state.Margin = new Padding(3, 8, 3, 8);
        layout.Controls.Add(state, 0, 2);
        var movement = Row();
        movement.Controls.Add(new Label { Text = "统一动作开度 A", AutoSize = true, Margin = new Padding(3, 10, 3, 3) });
        ConfigureNumber(opening, 0, 180, 2, 0.09M, 86M); movement.Controls.Add(opening);
        movement.Controls.Add(new Label { Text = "度   步长", AutoSize = true, Margin = new Padding(3, 10, 3, 3) });
        step.DropDownStyle = ComboBoxStyle.DropDownList; step.Width = 70;
        foreach (int value in new int[] { 1, 2, 5, 10, 20 }) step.Items.Add(value);
        step.SelectedIndex = 0; movement.Controls.Add(step);
        movement.Controls.Add(new Label { Text = "µs", AutoSize = true, Margin = new Padding(3, 10, 3, 3) });
        movement.Controls.Add(Button("双爪内收", delegate { NudgeOpening(-1); }));
        movement.Controls.Add(Button("双爪外展", delegate { NudgeOpening(1); }));
        layout.Controls.Add(movement, 0, 3);
        var left = Row();
        left.Controls.Add(new Label { Text = "左 S4 安装offset", AutoSize = true, Width = 165, Margin = new Padding(3, 10, 3, 3) });
        ConfigureNumber(leftOffset, -2000, 2000, 0, 1, 0); left.Controls.Add(leftOffset);
        left.Controls.Add(new Label { Text = "µs", AutoSize = true, Margin = new Padding(3, 10, 3, 3) });
        left.Controls.Add(Button("仅左内收", delegate { NudgeOffset(leftOffset, inverted ? 1 : -1); }));
        left.Controls.Add(Button("仅左外展", delegate { NudgeOffset(leftOffset, inverted ? -1 : 1); }));
        layout.Controls.Add(left, 0, 4);
        var right = Row();
        right.Controls.Add(new Label { Text = "右 S2 安装offset", AutoSize = true, Width = 165, Margin = new Padding(3, 10, 3, 3) });
        ConfigureNumber(rightOffset, -2000, 2000, 0, 1, 0); right.Controls.Add(rightOffset);
        right.Controls.Add(new Label { Text = "µs", AutoSize = true, Margin = new Padding(3, 10, 3, 3) });
        right.Controls.Add(Button("仅右内收", delegate { NudgeOffset(rightOffset, inverted ? -1 : 1); }));
        right.Controls.Add(Button("仅右外展", delegate { NudgeOffset(rightOffset, inverted ? 1 : -1); }));
        layout.Controls.Add(right, 0, 5);
        preview.AutoSize = true; preview.Margin = new Padding(3, 8, 3, 8);
        preview.Font = new Font("Consolas", 11F); layout.Controls.Add(preview, 0, 6);
        var saveRow = Row();
        saveRow.Controls.Add(Button("保存标定", delegate { SaveCalibration(); }));
        saveRow.Controls.Add(Button("应用已保存姿态", delegate { ApplySaved(); }));
        saveRow.Controls.Add(Button("重新读板上脉宽", delegate { AdoptLive(); }));
        referenceConfirmed.Text = "已人工确认安装基准/对称性"; referenceConfirmed.AutoSize = true;
        referenceConfirmed.Margin = new Padding(10, 8, 3, 3); saveRow.Controls.Add(referenceConfirmed);
        layout.Controls.Add(saveRow, 0, 7);
        layout.Controls.Add(Note("A减小=两侧内收。左右单调按钮只改offset；正offset=PWM脉宽增加。\n1 µs≈0.09°是指令分度，不保证实物每步都响应；0°只是内收参考，按实物可用范围操作。"), 0, 8);
        history.Multiline = true; history.ReadOnly = true; history.ScrollBars = ScrollBars.Vertical;
        history.Dock = DockStyle.Fill; history.Font = new Font("Consolas", 9F); layout.Controls.Add(history, 0, 9);

        opening.ValueChanged += delegate { ValuesChanged(); };
        leftOffset.ValueChanged += delegate { ValuesChanged(); };
        rightOffset.ValueChanged += delegate { ValuesChanged(); };
        FormClosing += delegate { DisconnectBoard(); session.Dispose(); };
        LoadSaved(); UpdatePreview();
        Log("先连接读取当前姿态。初次折算offset只是保持原姿态的起点，不是自动测得的安装误差。");
    }

    private static FlowLayoutPanel Row()
    {
        return new FlowLayoutPanel { AutoSize = true, Dock = DockStyle.Fill, WrapContents = true, Padding = new Padding(0, 2, 0, 2) };
    }
    private static Label Note(string text)
    {
        return new Label { Text = text, AutoSize = true, MaximumSize = new Size(900, 0), Margin = new Padding(3, 5, 3, 8) };
    }
    private Button Button(string text, Action click)
    {
        var button = new Button { Text = text, AutoSize = true, Padding = new Padding(6, 4, 6, 4), Margin = new Padding(4) };
        button.Click += delegate { Execute(click); }; return button;
    }
    private static void ConfigureNumber(NumericUpDown control, decimal min, decimal max, int decimals, decimal increment, decimal value)
    {
        control.Minimum = min; control.Maximum = max; control.DecimalPlaces = decimals;
        control.Increment = increment; control.Value = value; control.Width = 110; control.Margin = new Padding(4, 8, 4, 4);
    }
    private void Execute(Action action)
    {
        try { action(); }
        catch (Exception error) { Log("错误：" + error.Message); MessageBox.Show(this, error.Message, "未继续调节，保持最后脉宽"); }
    }
    private void Log(string text)
    {
        history.AppendText(DateTime.Now.ToString("HH:mm:ss.fff") + " " + text + Environment.NewLine);
        session.WriteLine(json.Serialize(new Dictionary<string, object> { { "time", DateTimeOffset.Now.ToString("o") }, { "message", text } }));
    }
    private int StepUs { get { return Convert.ToInt32(step.SelectedItem, CultureInfo.InvariantCulture); } }
    private int OpeningCdeg { get { return (int)(opening.Value * 100M); } }
    private int BaseLeft { get { return 500 + (inverted ? 18000 - OpeningCdeg : OpeningCdeg) / 9; } }
    private int BaseRight { get { return 500 + (inverted ? OpeningCdeg : 18000 - OpeningCdeg) / 9; } }
    private int LeftPulse { get { return BaseLeft + (int)leftOffset.Value; } }
    private int RightPulse { get { return BaseRight + (int)rightOffset.Value; } }
    private void UpdatePreview()
    {
        preview.Text = String.Format(CultureInfo.InvariantCulture,
            "目标PWM L(S4)={0}us  R(S2)={1}us    最近表读回 L={2}us R={3}us\nS逻辑等效 L≈{4:0.00}° R≈{5:0.00}°（offset另算）",
            LeftPulse, RightPulse, liveLeft, liveRight, opening.Value, 180M - opening.Value);
    }
    private void ValuesChanged()
    {
        if (loading) return;
        UpdatePreview();
        if (connected) Execute(ApplyCurrent);
    }
    private void NudgeOpening(int direction)
    {
        if (!connected) throw new InvalidOperationException("先连接读取。");
        opening.Value = Math.Max(opening.Minimum, Math.Min(opening.Maximum, opening.Value + direction * StepUs * 0.09M));
    }
    private void NudgeOffset(NumericUpDown control, int direction)
    {
        if (!connected) throw new InvalidOperationException("先连接读取。");
        control.Value = Math.Max(control.Minimum, Math.Min(control.Maximum, control.Value + direction * StepUs));
    }

    private string ConsoleCommand(string command, string expected)
    {
        serial.DiscardInBuffer();
        serial.Write("\r\n" + command + "\r\n");
        var response = new StringBuilder();
        var clock = Stopwatch.StartNew();
        while (clock.ElapsedMilliseconds < 2200)
        {
            response.Append(serial.ReadExisting());
            string text = response.ToString();
            if (Regex.IsMatch(text, @"(?m)^(ERR(?:\s|$)|BUSY:)")) throw new IOException("板端拒绝 " + command + ": " + text);
            if (Regex.IsMatch(text, expected)) return text;
            Thread.Sleep(15);
        }
        throw new IOException("未收到 " + command + " 回执：" + response);
    }
    private void OpenConsole()
    {
        serial = new SerialPort(port.Text.Trim(), 115200, Parity.None, 8, StopBits.One);
        serial.DtrEnable = false; serial.RtsEnable = false; serial.WriteTimeout = 1000;
        serial.Open();
        string pong = ConsoleCommand("PING", "PONG");
        string report = ConsoleCommand("STAT", @"(?m)^(?:DRIVE TARGET_MM_S=[^\r\n]+|MOTOR 3 [^\r\n]+)\r?$" );
        consoleIdentity = pong + report;
        motionService = report.Contains("BOARD=F405_ROBO SERVICE=MOTIONDEBUG");
        if (motionService)
        {
            if (!Regex.IsMatch(report, @"STATUS STATE=(?:IDLE|DONE|STOPPED)\b"))
                throw new IOException("MotionDebug动作仍在执行，请先停止运动再标定。");
        }
        else if (pong.Contains("PONG NORMALDEBUG F405"))
        {
            ConsoleCommand("DEBUG", "DEBUG OK");
        }
        else throw new IOException("未识别到F405的MotionDebug或Task维护控制台。");
        Log(motionService ? "已识别MotionDebug；只读取/调整双爪。" : "已用原DEBUG进入Task维护，比赛任务暂停。");
    }
    private static string Quote(string text)
    {
        if (text.IndexOf('"') >= 0) throw new ArgumentException("路径中不能含双引号。");
        return "\"" + text.TrimEnd('\\') + "\"";
    }
    private void AppendHelperLog(object sender, DataReceivedEventArgs args)
    {
        if (args.Data == null) return;
        lock (helperLogLock)
        {
            helperLog.AppendLine(args.Data);
            if (helperLog.Length > 16000) helperLog.Remove(0, helperLog.Length - 12000);
        }
    }
    private string HelperTail()
    {
        lock (helperLogLock) { return helperLog.ToString(); }
    }
    private void StartHelper()
    {
        int rpcPort;
        var listener = new TcpListener(IPAddress.Loopback, 0);
        listener.Start(); rpcPort = ((IPEndPoint)listener.LocalEndpoint).Port; listener.Stop();
        string scripts = Path.GetFullPath(Path.Combine(Path.GetDirectoryName(openOcdExe), "../share/openocd/scripts"));
        string args = "-s " + Quote(scripts) + " -f " + Quote(Path.Combine(repo, "dap.cfg")) +
            " -c " + Quote("bindto 127.0.0.1") + " -c " + Quote("tcl port " + rpcPort) +
            " -c " + Quote("gdb port disabled") + " -c " + Quote("telnet port disabled") +
            " -c " + Quote("stm32f4x.cpu configure -event examine-end {}") +
            " -f " + Quote(Path.Combine(repo, "tools/F405ClawCalibrator.tcl"));
        lock (helperLogLock) { helperLog.Clear(); }
        helper = new Process();
        helper.StartInfo = new ProcessStartInfo(openOcdExe, args) {
            WorkingDirectory = repo, UseShellExecute = false, CreateNoWindow = true,
            WindowStyle = ProcessWindowStyle.Hidden, RedirectStandardError = true, RedirectStandardOutput = true
        };
        helper.OutputDataReceived += AppendHelperLog; helper.ErrorDataReceived += AppendHelperLog;
        helper.Start(); helper.BeginOutputReadLine(); helper.BeginErrorReadLine();
        var clock = Stopwatch.StartNew();
        while (clock.ElapsedMilliseconds < 6000)
        {
            if (helper.HasExited) throw new IOException("SWD连接失败：" + HelperTail());
            var attempt = new TcpClient();
            try
            {
                attempt.Connect(IPAddress.Loopback, rpcPort);
                attempt.NoDelay = true;
                rpcClient = attempt; rpcStream = attempt.GetStream();
                rpcStream.ReadTimeout = 2500; rpcStream.WriteTimeout = 2500;
                return;
            }
            catch (SocketException) { attempt.Close(); Thread.Sleep(80); }
        }
        throw new IOException("未连接到本窗口的OpenOCD：" + HelperTail());
    }
    private string Rpc(string command)
    {
        if (rpcStream == null) throw new IOException("SWD未连接。");
        byte[] request = Encoding.UTF8.GetBytes(command + (char)26);
        rpcStream.Write(request, 0, request.Length);
        var response = new List<byte>();
        while (response.Count < 32768)
        {
            int value = rpcStream.ReadByte();
            if (value < 0) throw new IOException("OpenOCD连接已关闭。");
            if (value == 26) return Encoding.UTF8.GetString(response.ToArray()).Trim();
            response.Add((byte)value);
        }
        throw new IOException("OpenOCD回执长度异常。");
    }
    private int[] ParsePulses(string text)
    {
        Match match = Regex.Match(text, @"^OK (\d+) (\d+)$");
        if (!match.Success) throw new IOException(text);
        return new int[] { int.Parse(match.Groups[1].Value), int.Parse(match.Groups[2].Value) };
    }
    private void ConnectBoard()
    {
        DisconnectBoard();
        try
        {
            OpenConsole(); StartHelper(); connected = true;
            AdoptLive();
            state.Text = "已连接 " + serial.PortName + "；双爪倒装=" + inverted + "；只在手动调节时写脉宽表。";
        }
        catch { DisconnectBoard(); throw; }
    }
    private void AdoptLive()
    {
        if (!connected) throw new IOException("先连接读取。");
        int[] values = ParsePulses(Rpc("claw_read"));
        liveLeft = values[0]; liveRight = values[1];
        loading = true;
        try { leftOffset.Value = liveLeft - BaseLeft; rightOffset.Value = liveRight - BaseRight; }
        finally { loading = false; }
        hasRead = true; UpdatePreview();
        Log(String.Format("读回左{0}us/右{1}us；以A={2:0.00}°折算起点offset，未移动爪。", liveLeft, liveRight, opening.Value));
    }
    private void ApplyCurrent()
    {
        if (!connected) throw new IOException("先连接读取。");
        if (LeftPulse < 500 || LeftPulse > 2500 || RightPulse < 500 || RightPulse > 2500)
            throw new ArgumentOutOfRangeException("脉宽", "目标超过原500～2500µs范围，未写入。");
        int[] values = ParsePulses(Rpc("claw_set " + LeftPulse + " " + RightPulse));
        liveLeft = values[0]; liveRight = values[1]; UpdatePreview();
        Log(String.Format("A={0:0.00}°，左offset={1}us/右offset={2}us；表读回={3}/{4}us。",
            opening.Value, leftOffset.Value, rightOffset.Value, liveLeft, liveRight));
    }
    private void RestoreOldClosed()
    {
        DisconnectBoard();
        try
        {
            OpenConsole();
            foreach (string command in new string[] { "S 2 72", "S 4 86", "S 2 90" })
            {
                string reply = ConsoleCommand(command, motionService ? @"SERVO ID=\d+ TARGET_DEG=\d+ PWM=ON" : @"(?m)^OK\r?$");
                Log(command + " 已确认。"); Thread.Sleep(400);
            }
            loading = true; opening.Value = 86M; referenceConfirmed.Checked = false; loading = false;
            StartHelper(); connected = true; AdoptLive();
            state.Text = "已恢复旧合爪；A=86°为微调起点，offset待按实物确认。";
        }
        catch { DisconnectBoard(); throw; }
    }
    private void DisconnectBoard()
    {
        connected = false;
        if (rpcStream != null) { try { Rpc("shutdown"); } catch { } rpcStream.Dispose(); rpcStream = null; }
        if (rpcClient != null) { rpcClient.Close(); rpcClient = null; }
        if (helper != null)
        {
            try { if (!helper.WaitForExit(700)) helper.Kill(); } catch { }
            helper.Dispose(); helper = null;
        }
        if (serial != null) { serial.Dispose(); serial = null; }
        state.Text = "已断开，保持最后脉宽；重启或新S命令会覆盖临时微调。";
    }

    private int SavedInt(string key)
    {
        if (saved == null || !saved.ContainsKey(key)) throw new IOException("保存文件缺少 " + key);
        return Convert.ToInt32(saved[key], CultureInfo.InvariantCulture);
    }
    private void LoadSaved()
    {
        string path = Path.Combine(dataDirectory, "calibration.json");
        if (!File.Exists(path)) return;
        try
        {
            saved = json.Deserialize<Dictionary<string, object>>(File.ReadAllText(path));
            loading = true;
            opening.Value = SavedInt("opening_cdeg") / 100M;
            loading = false;
            Log("发现已保存参数；不会自动应用，连接后可点“应用已保存姿态”。");
        }
        catch (Exception error) { loading = false; saved = null; Log("保存文件未载入：" + error.Message); }
    }
    private void SaveCalibration()
    {
        if (!hasRead) throw new IOException("先连接读取当前姿态。");
        if (connected)
        {
            int[] values = ParsePulses(Rpc("claw_read"));
            liveLeft = values[0]; liveRight = values[1]; UpdatePreview();
        }
        if (LeftPulse != liveLeft || RightPulse != liveRight)
            throw new IOException("当前面板目标与最后读回不同，请先应用或重新读取，再保存。");
        saved = new Dictionary<string, object> {
            { "schema_version", 1 }, { "saved_at", DateTimeOffset.Now.ToString("o") },
            { "board", "F405" }, { "calibration_kind", "manual_claw_pulse_adjustment" },
            { "opening_cdeg", OpeningCdeg }, { "opening_deg", opening.Value },
            { "left_offset_us", (int)leftOffset.Value }, { "right_offset_us", (int)rightOffset.Value },
            { "left_pulse_us", liveLeft }, { "right_pulse_us", liveRight },
            { "pulse_quantum_us", 1 }, { "pulse_per_degree_nominal", 2000.0 / 180.0 },
            { "claws_inverted", inverted }, { "spline_teeth_user_reported", 25 },
            { "installation_reference_user_confirmed", referenceConfirmed.Checked },
            { "mechanical_angle_sensor_feedback", false }, { "permanent_firmware_offsets_written", false },
            { "running_console_identity_raw", consoleIdentity }
        };
        string path = Path.Combine(dataDirectory, "calibration.json");
        File.WriteAllText(path, json.Serialize(saved), new UTF8Encoding(false));
        string summary = String.Format(CultureInfo.InvariantCulture,
            "F405双爪人工微调结果\r\n保存时间：{0}\r\n统一动作开度：{1:0.00}°\r\n左S4 offset：{2} µs\r\n右S2 offset：{3} µs\r\n左/右PWM：{4}/{5} µs\r\n人工确认安装基准/对称性：{6}\r\n未写入固件；重启或新S指令可能覆盖临时脉宽。\r\n",
            DateTimeOffset.Now.ToString("o"), opening.Value, leftOffset.Value, rightOffset.Value,
            liveLeft, liveRight, referenceConfirmed.Checked);
        File.WriteAllText(Path.Combine(dataDirectory, "当前标定.txt"), summary, new UTF8Encoding(true));
        Log("已保存：" + path + "；固件未修改。");
    }
    private void ApplySaved()
    {
        if (!connected) throw new IOException("先连接读取。");
        if (saved == null) throw new IOException("尚无calibration.json，先完成微调并保存。");
        if (!saved.ContainsKey("claws_inverted") || Convert.ToBoolean(saved["claws_inverted"]) != inverted)
            throw new IOException("保存文件的倒装设定与当前配置不同，请按当前安装重新标定。");
        loading = true;
        try
        {
            opening.Value = SavedInt("opening_cdeg") / 100M;
            leftOffset.Value = SavedInt("left_offset_us");
            rightOffset.Value = SavedInt("right_offset_us");
            referenceConfirmed.Checked = saved.ContainsKey("installation_reference_user_confirmed") &&
                Convert.ToBoolean(saved["installation_reference_user_confirmed"]);
        }
        finally { loading = false; }
        ApplyCurrent();
    }
}
