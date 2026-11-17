using System;
using System.IO;
using System.Reflection;
using System.Runtime.Serialization;
using System.Windows.Forms;
class ServoParityCheck {
 static readonly BindingFlags F=BindingFlags.Instance|BindingFlags.NonPublic;
 static void Set(object o,string n,object v){o.GetType().GetField(n,F).SetValue(o,v);}
 [STAThread] static void Main(string[] args){
  object f=FormatterServices.GetUninitializedObject(typeof(CalibratorForm));
  Set(f,"repo",Path.GetFullPath(args[0]));
  typeof(CalibratorForm).GetMethod("ReloadSourceConfiguration",F).Invoke(f,null);
  var define=typeof(CalibratorForm).GetMethod("Define",F);
  var calc=typeof(CalibratorForm).GetMethod("ConfigPulse",F);
  var opening=new NumericUpDown{Minimum=0,Maximum=180};
  var left=new NumericUpDown{Minimum=-2000,Maximum=2000,Value=Convert.ToDecimal(define.Invoke(f,new object[]{"APP_SERVO_S4_OFFSET_US"}))};
  var right=new NumericUpDown{Minimum=-2000,Maximum=2000,Value=Convert.ToDecimal(define.Invoke(f,new object[]{"APP_SERVO_S2_OFFSET_US"}))};
  Set(f,"opening",opening);Set(f,"leftOffset",left);Set(f,"rightOffset",right);
  for(int id=1;id<=4;id++) for(int a=0;a<=180;a++){
   int pulse=(int)calc.Invoke(f,new object[]{id,a});
   Console.WriteLine(id+" "+a+" "+pulse);
   if(id==2||id==4){
    opening.Value=id==4?a:180-a;
    var prop=typeof(CalibratorForm).GetProperty(id==4?"LeftPulse":"RightPulse",F);
    if((int)prop.GetValue(f,null)!=pulse)throw new Exception("Claw panel mismatch "+id+" "+a);
   }
  }
  var presets=(System.Collections.IEnumerable)typeof(CalibratorForm).GetMethod("Presets",F).Invoke(f,null);
  foreach(object p in presets){
   var type=p.GetType();string name=(string)type.GetField("Name").GetValue(p);int lift=(int)type.GetField("Lift").GetValue(p);
   if(name=="合爪复审"&&lift!=85)throw new Exception("AUDIT lift mismatch");
   if(name=="放下后抬臂"&&lift!=65)throw new Exception("EXIT lift mismatch");
  }
 }
}

