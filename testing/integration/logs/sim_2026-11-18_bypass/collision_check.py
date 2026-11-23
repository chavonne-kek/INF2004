import sys,json,math,glob,os
# car body: from 60 mm behind the axle to 100 mm ahead, 75 mm each side
def corners(x,y,th):
    c,s=math.cos(math.radians(th)),math.sin(math.radians(th))
    pts=[]
    for fx in (-60,100):
        for fy in (-75,75):
            pts.append((x+fx*c-fy*s, y+fx*s+fy*c))
    # also edge midpoints
    for fx in (-60,20,100):
        for fy in (-75,0,75):
            pts.append((x+fx*c-fy*s, y+fx*s+fy*c))
    return pts
for d in sys.argv[1:]:
    for f in sorted(glob.glob(d+'/*.out')):
        n=os.path.basename(f)[:-4]
        scen=[l.split('|',1)[1] for l in open(os.path.dirname(os.path.abspath(f))+'/../'+('scen1.txt' if 'res_fix/' in f+'/' or d.endswith('res_fix') or d.endswith('res_base') else 'scen.txt')) if l.startswith(n+'|')]
        j=json.loads(scen[0]) if scen else {}
        bx=j.get('boxX',-1); bd=j.get('boxD',100); bw=j.get('boxW',100); by=j.get('boxY',0)
        x0,x1,y0,y1=bx,bx+bd,by-bw/2,by+bw/2
        worst=1e9; hit=0
        for l in open(f):
            if not l.startswith('T '): continue
            a=l.split(); x,y,th=float(a[2]),float(a[3]),float(a[4])
            for (px,py) in corners(x,y,th):
                dx=max(x0-px,0,px-x1); dy=max(y0-py,0,py-y1)
                dist=math.hypot(dx,dy)
                if dist==0: hit+=1
                worst=min(worst,dist)
        print(f"{d}/{n}: closest car-box gap {worst:.0f} mm, samples inside {hit}")
