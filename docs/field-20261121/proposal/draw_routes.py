"""Generate the route-review sheets. No firmware dependency; all world units are mm."""
from math import hypot
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

OUT = Path(__file__).parent
FONT = "C:/Windows/Fonts/msyh.ttc"
GREEN, GREY, RED, BLUE = "#08745b", "#596876", "#b23d35", "#185a96"
INK, GRID = "#192e3a", "#d8e1e5"

def render(blue, n, title, lines, points, notes, sub):
    im = Image.new("RGB", (1190, 650), "white")
    d = ImageDraw.Draw(im)
    def txt(x,y,s,size=19,color=INK,anchor="la"):
        d.text((x,y),s,font=ImageFont.truetype(FONT,size),fill=color,anchor=anchor)
    side = "蓝" if blue else "红"
    txt(35,16,f"{n:02d}   {title}",30)
    txt(35,65,side+"方实例  /  "+sub,18,GREY)
    # Detail envelope: a half-field plus centre; blue is the true 180-degree map rotation.
    scale=.33
    def xy(p):
        x,y=p
        if blue: x,y=-x,-y
        return 361+x*scale, (133 if blue else 568)-y*scale
    def rect(p,q,fill,outline):
        a,b=xy(p),xy(q)
        d.rectangle((min(a[0],b[0]),min(a[1],b[1]),max(a[0],b[0]),max(a[1],b[1])),fill,outline,2)
    rect((-965,-105),(965,1290),"#f5f8f9",GRID)
    for x in (-400,0,400):
        d.line([xy((x,-90)),xy((x,1280))],fill=GRID,width=1)
    d.line([xy((-960,0)),xy((960,0))],fill="#9cabb3",width=2)
    rect((-330,1140),(330,1290),"#dceaf6" if blue else "#f8e1e0",GRID)
    rect((-300,1170),(0,1290),"#d3eadd",GRID)
    txt(*xy((-150,1245)),"物资",17,INK,"mm")
    txt(*xy((150,1245)),"伤员",17,GREY,"mm")
    for x in (-400,250):
        rect((x,560),(x+150,660),"#efca48","#86670c")
    for style, ps in lines:
        color = {"old":GREY,"new":GREEN,"risk":RED,"local":BLUE}[style]
        for a,b in zip(map(xy,ps),map(xy,ps[1:])):
            dx,dy=b[0]-a[0],b[1]-a[1]; L=hypot(dx,dy)
            if L<1:continue
            ux,uy=dx/L,dy/L
            if style in ("new","risk"):
                for j in range(0,int(L)+1,12):
                    k=min(L,j+8)
                    d.line((a[0]+ux*j,a[1]+uy*j,a[0]+ux*k,a[1]+uy*k),color,4)
            else:d.line((a,b),color,3)
            if L>16:
                cx,cy=a[0]+dx*.55,a[1]+dy*.55
                d.polygon([(cx+6*ux,cy+6*uy),(cx-5*ux-4*uy,cy-5*uy+4*ux),
                           (cx-5*ux+4*uy,cy-5*uy-4*ux)],fill=color)
    for key,p in points:
        x,y=xy(p)
        d.ellipse((x-10,y-10,x+10,y+10),fill="white",outline=INK,width=2)
        txt(x,y,key,13,INK,"mm")
    txt(725,112,"读图顺序",22)
    yy=161
    for heading,detail in notes:
        txt(725,yy,heading,21,GREEN); yy+=35
        for row in detail.split("|"):
            txt(725,yy,row,17); yy+=27
        yy+=18
    for i,(c,t) in enumerate([(GREY,"旧路线"),(GREEN,"提议路线"),(BLUE,"机械动作 / 同类专线"),(RED,"风险 / 拟删除")]):
        txt(40+[0,145,305,550][i],612,"━ "+t,17,c)
    txt(1150,615,"局部放大 · 点为定位参考点，非车身外廓",15,GREY,"ra")
    im.save(OUT/f"{'blue' if blue else 'red'}-{n}.png",optimize=True)

C,P,O,S,T=(0,300),(-150,740),(-150,540),(-400,0),(400,0)
D,E=(-700,350),(-700,550)
def retract(p):
    l=hypot(*p); return tuple(q*(1-350/l) for q in p)
B,A=retract(D),retract(E)
I=(-150,1210); U=(-150,910); K=(0,910); Z=(0,0)
W,V=(-560,910),(-560,760)
sheets=[
("防守暂存：放下、直退、直返",[
 ("old",[Z,E,A,S]),("new",[Z,D]),("local",[D,B]),("new",[B,S])],
 [("D",D),("E",E),("B",B),("1",S),("3",T)],
 [("E → D  只比较暂存点","旧点距横向中线 550，候选改为 350。|两方案 x 均距纵向中线 700。"),
 ("D → B  沿放手时车头倒退 350","图按从中心直进且车头沿该线示例。|实际 B 随放手位置和朝向变动。"),
 ("B → ①  仍走直线","①/③ 位置不改；换左右出发位，|暂存点和①一起左右镜像。")],"示例：①在本方图示这一侧；不是所有抓取起点的保证"),
("普通送物：中继、候选点、门口对准",[
 ("old",[S,O]),("new",[S,C,P]),("local",[P,I])],
 [("1",S),("C",C),("P",P),("O",O),("I",I)],
 [("C  场心向本方 300","从实际抓取位前往 C，图以①附近举例。|半径 50 内切下一段，不停下来追中心。"),
 ("P  围栏前 400","物资半区的 x 不改。先正对围栏，|再看物资分区；不要对着整框中心。"),
 ("C → P  是斜线，不是沿中线穿门","中心最近只余 120 mm。若外廓不够，|应另画中线过障后再分侧，不默许直穿。")],"灰 O 为围栏前 600 的旧点；绿色为请求方案，非实测"),
("放货：机械动作的场地位置",[
 ("old",[O,(-150,940),I]),("new",[P,(-150,940)]),("local",[(-150,940),I]),("risk",[(-105,1210),(-105,910)])],
 [("O",O),("P",P),("L",(-150,940)),("I",I),("U",U)],
 [("P → L  拟走 200，再调到 83°","旧为 O 起走 400；把起点前移 200，|同一名义变舵位置 L 不变。"),
 ("L → I  保航向盲推 270","合计由 670 改为 470。均为名义轮程，|提前确认、超时或轮滑不等于到了 I。"),
 ("I → U  正常退出后退 300","抬舵机、倒退、回运输姿态。|退线为可读性稍作横向错开。")],"普通无侧挤名义行程；原地补推、提前退出另核"),
("返搜：要横移到哪条线才倒退？",[
 ("old",[U,S]),("risk",[U,(-400,910),S]),("new",[U,K,Z,S]),("new",[Z,T])],
 [("U",U),("K",K),("0",Z),("1",S),("3",T)],
 [("红虚线  直接横到目标搜索列","x=±400 正好是黄块外沿。|再直倒时中心线擦边，不建议。"),
 ("绿虚线  先横到中缝，再倒过障","K 在 x=0，沿中线倒回 y=0；|最后横到运送前记住的①或③。"),
 ("U 是示例，不是固定出区坐标","按实际退出点启动。若已在障碍带内，|不能假定第一段横移一定有空间。")],"灰为旧直返示例；两条绿色末段择一，不连扫两站"),
("同类挡门：取走、投送、找回原货",[
 ("old",[O,(50,540),O]),("local",[P,(50,740),P]),
 ("new",[P,(-75,870)]),("new",[(-75,870),P]),("local",[P,I])],
 [("P",P),("B",(50,740)),("M",(-75,870)),("I",I),("O",O)],
 [("P → B  临时放原货","右转 90°，前进 200，放货；|后退 200，左转复朝安全区。"),
 ("M  抓真正可送的挡路物资","M 为示例，120° 新图实抓、闭爪复审。|建议门口局部任务不绕回 C。"),
 ("抓完 → P → I → 回到原记的点","先回相应物资预备点复核、转正、|看框对准，再送；退出后找回原货。")],"蓝为动作；旧/新放货位置按实际偏移；不是把黄台搬走"),
("异类／首件等需横扫：只保留 W1",[
 ("new",[P,(50,740)]),("local",[(50,740),(-560,740),W]),
 ("new",[W,(440,910)]),("risk",[V,(440,760)]),
 ("local",[(440,910),(-150,910),P])],
 [("P",P),("B",(50,740)),("1",W),("2",V)],
 [("①  保留第一档，横扫 1000","在右转后的坐标系，先退到扫障列，|再移到扫障线，张爪横扫。"),
 ("②  去掉 B3/B4 第二档","旧第二档比第一档向场心偏 150。|二者原本二选一，不是连续两扫。"),
 ("扫完抬起、回预备点、找回原货","新 P 起横向接近 W1 的这段，|中心到块仅 80 mm；需验外廓。")],"右转放货→退到扫障列→横移到起点；①②不是搜索点")]

for mirrored in (False, True):
    for i, args in enumerate(sheets,1):
        render(mirrored,i,*args)

def overview():
    im=Image.new("RGB",(1090,735),"white"); d=ImageDraw.Draw(im)
    def text(x,y,s,size=18,color=INK,anchor="la"):
        d.text((x,y),s,font=ImageFont.truetype(FONT,size),fill=color,anchor=anchor)
    def xy(x,y): return 340+x*.20,393-y*.20
    def box(x,y,X,Y,fill,edge):
        d.rectangle((*xy(x,Y),*xy(X,y)),fill,edge,2)
    text(34,13,"决赛路线确认   /   全场索引",30)
    text(35,58,"场地 3000 × 3000 mm，红方在上；图纸尺寸，尚无实车外廓。",18,GREY)
    box(-1500,-1500,1500,1500,"#f5f8f9",GREY)
    d.line([xy(0,1500),xy(0,-1500)],GRID,2);d.line([xy(-1500,0),xy(1500,0)],GRID,2)
    for sign in (1,-1):
        for x in (-400,250):
            ys=sorted([sign*560,sign*660]);box(x,ys[0],x+150,ys[1],"#efca48","#86670c")
        ys=sorted([1140*sign,1500*sign]);box(-330,ys[0],330,ys[1],"#f8e1e0" if sign==1 else "#dceaf6",GREY)
        text(*xy(-150*sign,1330*sign),"物资",15,INK,"mm")
        text(*xy(150*sign,1330*sign),"伤员",15,GREY,"mm")
        for key,(x,y) in (("C",C),("P",P),("W1",W)):
            u,v=xy(sign*x,sign*y);d.ellipse((u-5,v-5,u+5,v+5),fill=GREEN);text(u+8,v-10,key,16,GREEN)
    for z,(x,y) in enumerate([(-1350,1350),(1350,1350),(-1350,-1350),(1350,-1350)],1):
        box(x-150,y-150,x+150,y+150,"#e3e7e8",GRID);text(*xy(x,y),str(z)+"区",16,GREY,"mm")
    for x in (-400,400):
        u,v=xy(x,0);d.ellipse((u-5,v-5,u+5,v+5),fill=BLUE);text(u,v+8,"搜索点",15,BLUE,"ma")
    text(688,117,"看图时认三个位置",24)
    for k,(a,b,c) in enumerate([
      ("C   中继点","沿中线，离场心 300；进半径 50","切下一段，不在这里做视觉对正。"),
      ("P   物资候选点","离己方围栏面 400，红左蓝右；","停车核载、转正，再看物资分区。"),
      ("W1   推扫起点","首档保留，第二档拟删除。","同类抓送不经过 W1。"),
      ("黄色   固定障碍","每块 150 × 100，高 25。","不因误识别为伤员而主动抓取。"),
      ("① / ③   搜索点不动","1/3 区出发时①在左；2/4 区在右。","正式送完要回本批记住的①或③。")]):
        y=181+k*102;text(688,y,a,20,GREEN);text(688,y+33,b,17);text(688,y+58,c,17)
    im.save(OUT/"field.png",optimize=True)

overview()
