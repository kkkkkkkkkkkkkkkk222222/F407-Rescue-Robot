"""Defense review atlas. World coordinates in mm; no control-system code."""
import math
import sys
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

OUT = Path(__file__).resolve().parent
sys.path.insert(0, str(OUT.parents[1]))
from check_clearance import gap

INK, SUB, GO, BACK, HOME, OPEN = "#132e3c", "#495f6c", "#087259", "#aa4824", "#165f9a", "#836b99"
PAPER, GRID = "#f6f9fa", "#cfdbdf"
fonts = {}
def text(d, x, y, msg, size=20, color=INK, anchor="lt"):
    if size not in fonts:
        fonts[size] = ImageFont.truetype("C:/Windows/Fonts/msyh.ttc", size)
    d.text((x,y), msg, fill=color, font=fonts[size], anchor=anchor)

def arrow(d, a, b, color, width=4, dash=False):
    dx,dy=b[0]-a[0],b[1]-a[1]
    length=math.hypot(dx,dy)
    if length<1:return
    ux,uy=dx/length,dy/length
    if dash:
        for j in range(0,int(length),12):
            k=min(j+7,length)
            d.line((a[0]+j*ux,a[1]+j*uy,a[0]+k*ux,a[1]+k*uy),fill=color,width=width)
    else: d.line((a,b),fill=color,width=width)
    if length>22:
        x,y=a[0]+.54*dx,a[1]+.54*dy
        d.polygon([(x+8*ux,y+8*uy),(x-6*ux-5*uy,y-6*uy+5*ux),
                   (x-6*ux+5*uy,y-6*uy-5*ux)],fill=color)

def rect(d, xy, a,b,fill,outline):
    x,y=xy(a); X,Y=xy(b)
    d.rectangle((min(x,X),min(y,Y),max(x,X),max(y,Y)),fill,outline,2)

def sign(zone):return -1 if zone in (1,3) else 1
def geometry(zone, side, depth, pickup, yaw=None):
    end=(sign(zone)*700,side*depth); station=(sign(zone)*400,0)
    h=(math.atan2(end[1]-pickup[1], end[0]-pickup[0]) if yaw is None else math.radians(yaw))
    back=(end[0]-350*math.cos(h), end[1]-350*math.sin(h))
    return end, back, station, math.degrees(h)%360

def point(d, p, c, char):
    x,y=p; d.ellipse((x-12,y-12,x+12,y+12),fill="white",outline=c,width=3)
    text(d,x,y,char,15,c,"mm")

def blockfield(d,xy):
    for x in (-400,250):
        for y in (-660,560):rect(d,xy,(x,y),(x+150,y+100),"#ffd558","#976b00")

def pair(x):
    return f"({x[0]:.0f}, {x[1]:.0f})".replace("-0,", "0,").replace(", -0)",", 0)")

def detail(d, ox, zone, side, depth, pickup, yaw=None):
    xsign=sign(zone)
    # Fixed world-up (red-up); change only crop centre, not axis orientation.
    def xy(p):return (ox+278+(p[0]-xsign*340)*.48,438-(p[1]-side*260)*.48)
    d.rectangle((ox+10,195,ox+546,675),PAPER,GRID,2)
    for x in range(-1000,1001,100):
        u=xy((x,0))[0]
        if ox+11<u<ox+545:d.line((u,196,u,674),fill=GRID if x%400==0 else "#e9eef0")
    for y in range(-800,801,100):
        v=xy((0,y))[1]
        if 196<v<674:d.line((ox+11,v,ox+545,v),fill=GRID if y==0 else "#e9eef0")
    # Only obstacles visible in this crop, avoid marks outside the viewport.
    x=-400 if xsign<0 else 250; y=560 if side==1 else -660
    rect(d,xy,(x,y),(x+150,y+100),"#ffd558","#976b00")
    D,B,S,h=geometry(zone,side,depth,pickup,yaw)
    p1,p2=xy(D),xy(B)
    arrow(d,xy(pickup),p1,GO,5)
    # Small perpendicular display offset for the reverse stroke on a retraced line.
    dx,dy=p2[0]-p1[0],p2[1]-p1[1]; L=math.hypot(dx,dy); shift=(-7*dy/L,7*dx/L)
    arrow(d,(p1[0]+shift[0],p1[1]+shift[1]),(p2[0]+shift[0],p2[1]+shift[1]),BACK,4,True)
    arrow(d,p2,xy(S),HOME,5)
    for v,c,n in [(pickup,GO,"P"),(D,GO,"D"),(B,BACK,"B"),(S,HOME,"①")]:
        point(d,xy(v),c,n)
    # Triangle nose beyond the release point; it points opposite the reverse stroke.
    h=math.radians(h); q=(math.cos(h),-math.sin(h))
    tip=(p1[0]+29*q[0],p1[1]+29*q[1]);d.polygon([tip,(p1[0]+15*q[0]-5*q[1],p1[1]+15*q[1]+5*q[0]),
                (p1[0]+15*q[0]+5*q[1],p1[1]+15*q[1]-5*q[0])],fill=INK)
    text(d,ox+17,699,f"D {pair(D)}    B ≈ {pair(B)}    ① {pair(S)}",19)
    gaps=[gap(a,b) for a,b in [(pickup,D),(D,B),(B,S)]]
    text(d,ox+17,738,"中心到黄块最小净距（按图示位姿）",18,SUB)
    for i,(name,color,g) in enumerate(zip(["去程","倒退","返①"],[GO,BACK,HOME],gaps)):
        text(d,ox+17+i*175,771,f"{name}  {g:.0f} mm",19,color)
    return gaps

def startmap(d,zone,side):
    def xy(p):return 1302+p[0]*.088,340-p[1]*.088
    rect(d,xy,(-1500,-1500),(1500,1500),PAPER,GRID)
    for k in (-1,1):
        rect(d,xy,(-330,k*1140),(330,k*1500),"#efdada" if k>0 else "#cfdeec",GRID)
    blockfield(d,xy)
    sx,sy=sign(zone),1 if zone<3 else -1
    p0=1500-253/math.sqrt(2)
    def diag(v):return (sx*v,sy*v)
    a,b,e=[diag(p0-s/math.sqrt(2)) for s in (0,600,1700)]
    arrow(d,xy(a),xy(b),OPEN,3);arrow(d,xy(b),xy(e),OPEN,3)
    for k,p in [("起",a),("转",b),("观",e)]:point(d,xy(p),OPEN,k)
    D,B,S,_=geometry(zone,side,350,(0,0))
    d.line([xy((0,0)),xy(D),xy(B),xy(S)],fill=GO,width=2)
    text(d,1164,493,"起 → 转：先倒退 600",17,OPEN)
    text(d,1164,526,"转 → 观：转 180°，前进 1100",17,OPEN)
    text(d,1164,559,"观 → P：看物、接近、审核。",17)
    text(d,1164,590,"P 随真物变化，不画成定长。",17)

def sheet(zone,side,variant=False):
    team="红" if side==1 else "蓝";im=Image.new("RGB",(1480,895),"white"); d=ImageDraw.Draw(im)
    title="偏位抓取与放货航向：同一防守点，不同倒退终点" if variant else f"{team}方  ·  {zone} 区出发  /  开局防守暂存"
    text(d,34,24,title,32)
    text(d,35,80,"P 实际抓后出发点   →   D 暂存并放手   →   B 保持放手航向倒退 350 的终点   →   ① 首个搜索点",22,SUB)
    p=(160,-100) if variant else (0,0)
    for i,(ox,depth) in enumerate([(22,350 if variant else 550),(584,350)]):
        yaw=215 if variant and i==1 else None
        txt=(("同一 P，放货时朝向沿 P→D" if i==0 else "同一 P，假设放货时航向 215°")
             if variant else ("A   现行点｜纵向 550" if i==0 else "B   待确认点｜纵向 350"))
        text(d,ox+14,139,txt,23,GO if i else INK)
        detail(d,ox,zone,side,depth,p,yaw)
    text(d,1164,146,"全场位置 · 上红下蓝",19)
    startmap(d,zone,side)
    text(d,1164,652,"绿色实线   持物去 D",17,GO);text(d,1164,684,"橙色虚线   倒车离开 D",17,BACK)
    text(d,1164,716,"蓝色实线   空车直返①",17,HOME);text(d,1164,748,"黑色三角   放手时车头",17)
    text(d,36,827,("偏位与 215° 均为敏感性示例，不表示程序会主动转到 215°。" if variant else
                   "标准图假设 P=(0,0)、准确到 D、放手时车头沿 P→D；不是实车回放。"),19)
    text(d,36,858,"单位 mm；北向固定。橙线仅为避免遮住绿线作绘图错位。数值是定位点的间隙，不是轮、爪、载荷间隙。",17,SUB)
    return im

def overview():
    im=Image.new("RGB",(1304,810),"white");d=ImageDraw.Draw(im)
    text(d,30,18,"开局防守  /  红蓝双方 × 四个出发区",32)
    text(d,31,66,"候选 D=(±700, ±350)   ·   上红下蓝，八格方向一致   ·   拐点 B 由放手航向决定",19,SUB)
    for row,side in enumerate((1,-1)):
        for j in range(4):
            zone=j+1;ox=26+j*319;oy=108+row*319
            text(d,ox+6,oy,f"{'红' if side==1 else '蓝'}方 · {zone}区",21)
            def xy(p):return (ox+150+p[0]*.077,oy+157-p[1]*.077)
            rect(d,xy,(-1500,-1500),(1500,1500),PAPER,GRID)
            for k in (-1,1):rect(d,xy,(-330,1140*k),(330,1500*k),"#f0dbda" if k==1 else "#d8e6ed",GRID)
            blockfield(d,xy)
            sx,sy=sign(zone),1 if zone<3 else -1
            a=xy((sx*1321,sy*1321));e=xy((sx*119,sy*119))
            arrow(d,a,e,OPEN,3);text(d,a[0],a[1],"起",15,OPEN,"mm")
            D,B,S,_=geometry(zone,side,350,(0,0))
            arrow(d,xy((0,0)),xy(D),GO,4)
            aa,bb=xy(D),xy(B)
            arrow(d,(aa[0],aa[1]-4),(bb[0],bb[1]-4),BACK,3,True)
            arrow(d,bb,xy(S),HOME,3)
            for n,p,c in [("D",D,GO),("①",S,HOME)]:
                x,y=xy(p);d.ellipse((x-3,y-3,x+3,y+3),fill=c)
                text(d,x+(-5 if sx<0 else 5),y,n,15,c,"rm" if sx<0 else "lm")
            text(d,ox+6,oy+283,f"D {pair(D)}   ① {pair(S)}",16)
    text(d,32,762,"紫：发车向中心观察区    绿：抓后去防守点    橙：放手后倒退    蓝：返回①    黄：固定障碍",20,SUB)
    return im

if __name__=="__main__":
    for side,slug in [(1,"red"),(-1,"blue")]:
        for zone in range(1,5):
            sheet(zone,side).save(OUT/f"{slug}-{zone}.png",optimize=True)
    sheet(1,1,True).save(OUT/"offset.png",optimize=True)
    overview().save(OUT/"overview.png",optimize=True)
