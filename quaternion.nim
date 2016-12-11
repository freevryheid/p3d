import
  math,
  strutils,
  basic3d

const
  epsilon* = 0.00001f                  # floating point epsilon for single precision. todo: verify epsilon value and usage
  epsilonSquared* = epsilon * epsilon  # epsilon value squared

type
  Quaternion* = object
    x*,y*,z*,w*:float
  AA* = object  # angle/axis
    angle*: float
    axis*: Vector3d

proc
  quaternion*(x,y,z,w:float):Quaternion =
  Quaternion(x:x,y:y,z:z,w:w)

proc
  `*`*(q1,q2:Quaternion):Quaternion =
  quaternion(
    q1.w*q2.x+q1.x*q2.w+q1.y*q2.z-q1.z*q2.y,
    q1.w*q2.y+q1.y*q2.w+q1.z*q2.x-q1.x*q2.z,
    q1.w*q2.z+q1.z*q2.w+q1.x*q2.y-q1.y*q2.x,
    q1.w*q2.w-q1.x*q2.x-q1.y*q2.y-q1.z*q2.z
  )

proc
  `+`*(q1,q2:Quaternion):Quaternion =
  quaternion(q1.x+q2.x,q1.y+q2.y,q1.z+q2.z,q1.w+q2.w)

proc
  `+`*(v:Quaternion,r:float):Quaternion =
  quaternion(v.x+r,v.y,v.z,v.w)

proc
  `+`*(r:float,v:Quaternion):Quaternion =
  quaternion(v.x+r,v.y,v.z,v.w)

proc
  `*`*(v:Quaternion,r:float):Quaternion =
  quaternion(v.x*r,v.y*r,v.z*r,v.w*r)

proc
  `*`*(r:float,v:Quaternion):Quaternion =
  quaternion(v.x*r,v.y*r,v.z*r,v.w*r)

proc
  `-`*(v:Quaternion):Quaternion =
  quaternion(-v.x,-v.y,-v.z,-v.w)

proc
  `+=`*(p:var Quaternion, q:Quaternion) =
  p.x += q.x
  p.y += q.y
  p.z += q.z
  p.w += q.w

proc
  rtos(val:float):string =
  formatFloat(val,ffDefault,0)

proc
  `$`*(v:Quaternion):string =
  result=rtos(v.x)
  result.add(",")
  result.add(rtos(v.y))
  result.add(",")
  result.add(rtos(v.z))
  result.add(",")
  result.add(rtos(v.w))

proc
  norm*(v:Quaternion):float =
    sqrt(v.x*v.x+v.y*v.y+v.z*v.z+v.w*v.w)

proc
  normalize*(v:Quaternion):Quaternion =
  let sum = v.norm()
  quaternion(v.x/sum, v.y/sum, v.z/sum, v.w/sum)

proc
  conjugate*(v:Quaternion):Quaternion =
  quaternion(v.x,-v.y,-v.z,-v.w)

proc
  matrix*(v: Quaternion):Matrix3d =
  let (x, y, z, w) = (v.x, v.y, v.z, v.w)
  matrix3d(
    1-2*y*y-2*z*z,  2*x*y-2*w*z,  2*x*z+2*w*y,0,
      2*x*y+2*w*z,1-2*x*x-2*z*z,  2*y*z-2*w*x,0,
      2*x*z-2*w*y,  2*y*z+2*w*x,1-2*x*x-2*y*y,0,
                0,            0,            0,1
  )

proc
  `*`*(m,n: Matrix3d):Matrix3d =
  result.ax = m.ax*n.ax+m.ay*n.bx+m.az*n.cx+m.aw*n.tx
  result.ay = m.ax*n.ay+m.ay*n.by+m.az*n.cy+m.aw*n.ty
  result.az = m.ax*n.az+m.ay*n.bz+m.az*n.cz+m.aw*n.tz
  result.aw = m.ax*n.aw+m.ay*n.bw+m.az*n.cw+m.aw*n.tw
  result.bx = m.bx*n.ax+m.by*n.bx+m.bz*n.cx+m.bw*n.tx
  result.by = m.bx*n.ay+m.by*n.by+m.bz*n.cy+m.bw*n.ty
  result.bz = m.bx*n.az+m.by*n.bz+m.bz*n.cz+m.bw*n.tz
  result.bw = m.bx*n.aw+m.by*n.bw+m.bz*n.cw+m.bw*n.tw
  result.cx = m.cx*n.ax+m.cy*n.bx+m.cz*n.cx+m.cw*n.tx
  result.cy = m.cx*n.ay+m.cy*n.by+m.cz*n.cy+m.cw*n.ty
  result.cz = m.cx*n.az+m.cy*n.bz+m.cz*n.cz+m.cw*n.tz
  result.cw = m.cx*n.aw+m.cy*n.bw+m.cz*n.cw+m.cw*n.tw
  result.tx = m.tx*n.ax+m.ty*n.bx+m.tz*n.cx+m.tw*n.tx
  result.ty = m.tx*n.ay+m.ty*n.by+m.tz*n.cy+m.tw*n.ty
  result.tz = m.tx*n.az+m.ty*n.bz+m.tz*n.cz+m.tw*n.tz
  result.tw = m.tx*n.aw+m.ty*n.bw+m.tz*n.cw+m.tw*n.tw

let
  IDQUAT*:Quaternion = quaternion(1,0,0,0)
    ## Quick access to a identity quaternion

proc
  angleAxis*(v: Quaternion):AA =
  # convert quaternion to angle-axis.
  let
    squareLength: float = v.x*v.x+v.y*v.y+v.z*v.z
  if squareLength>epsilonSquared:
    result.angle = 2.0 * arccos(v.w)
    let inverseLength = 1.0 / pow(squareLength, 0.5)
    result.axis.x = v.x * inverseLength
    result.axis.y = v.y * inverseLength
    result.axis.z = v.z * inverseLength
  else:
    result.angle = 0.0
    result.axis.x = 1.0
    result.axis.y = 0.0
    result.axis.z = 0.0

when isMainModule:
  let
    q = quaternion(1, 2, 3, 4)
    q1 = quaternion(2, 3, 4, 5)
    q2 = quaternion(3, 4, 5, 6)
    r  = 7.0

  echo r+q
  assert(q1*q2!=q2*q1)


