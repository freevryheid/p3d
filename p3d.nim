import
  times,
  math,
  sdl2,
  sdl2/gfx,
  opengl,
  glu,
  basic3d,
  sphere

discard sdl2.init(INIT_VIDEO)

let
  width: cint = 1024
  height: cint = 768
  win = createWindow(
    "Physics",
    sdl2.SDL_WINDOWPOS_CENTERED, sdl2.SDL_WINDOWPOS_CENTERED,
    width, height,
    SDL_WINDOW_OPENGL or SDL_WINDOW_RESIZABLE)
  ctx = win.glCreateContext()
  dt = 0.001  # physics time step (s)
  COR = 0.25  # coef. of restitution

var
  evt = sdl2.defaultEvent
  fps: FpsManager
  run = true
  update = false
  Spheres: seq[Sphere] = @[]
  cyl: GLUquadric
  disk: GLUquadric
  zoom = 1.0
  bank = 0.1
  tilt = 0.1
  total_time = 0.0
  accumulator = 0.0
  newtime = 0.0
  currentTime = 0.0
  deltaTime = 0.0

proc
  reshape(newWidth: cint, newHeight: cint) =
  glViewport(0, 0, newWidth, newHeight)   # Set the viewport to cover the new window
  glMatrixMode(GL_PROJECTION)             # To operate on the projection matrix
  glLoadIdentity()                        # Reset
  gluPerspective(45.0, newWidth / newHeight, 0.1, 100.0)  # Enable perspective projection with fovy, aspect, zNear and zFar
  glMatrixMode(GL_MODELVIEW)

proc
  initGL() =
  var
    mat_specular: array[4, GLfloat] = [GLfloat(1.0), GLfloat(1.0), GLfloat(1.0), GLfloat(1.0)]
    mat_shininess: GLfloat = 50.0
    light_position: array[4, GLfloat] = [GLfloat(1.0), GLfloat(1.0), GLfloat(1.0), GLfloat(0.0)]
  loadExtensions()
  glClearColor(0.0,0.0,0.0,0.0)
  glMaterialfv(GL_FRONT, GL_SPECULAR, addr mat_specular[0])
  glMaterialfv(GL_FRONT, GL_SHININESS, addr mat_shininess)
  glLightfv(GL_LIGHT0, GL_POSITION, addr light_position[0])
  glEnable(GL_LIGHTING)
  glEnable(GL_LIGHT0)
  glClearDepth(1.0)
  glEnable(GL_DEPTH_TEST)
  glDepthFunc(GL_LEQUAL)
  glShadeModel(GL_SMOOTH)
  glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)
  fps.init()
  cyl = gluNewQuadric()
  disk = gluNewQuadric()
  gluQuadricDrawStyle(cyl, GLEnum(GLU_POINT))
  gluQuadricDrawStyle(disk, GLEnum(GLU_FILL))

proc
  mould() =
  glPushMatrix()
  gluCylinder(cyl, 0.075, 0.075, 0.15, 32, 32)
  gluDisk(disk,0,0.075, 32, 32)
  glPopMatrix()

proc
  doStuff() =
  while pollEvent(evt):
    case evt.kind
    of QuitEvent:
      run = false
    of KeyDown:
      if evt.key.keysym.sym == K_ESCAPE:
        run = false
      if evt.key.keysym.sym == K_SPACE:
        update = not update
    of MouseWheel:
      if evt.wheel.y > 0:
        zoom -= 0.1
        if zoom <= 0.4:
          zoom = 0.4
      else:
        zoom += 0.1
        if zoom >= 1.0:
          zoom = 1.0
    of MouseMotion:
      if evt.motion.xrel > 0:
        bank += 0.05
        if bank > 2:
          bank = 2
      else:
        bank -= 0.05
        if bank < 0:
          bank = 0.0
      if evt.motion.yrel > 0:
        tilt += 0.001
        if tilt > 1:
          tilt =1
      else:
        tilt -= 0.001
        if tilt < 0:
          tilt = 0.0
    else: discard
  # render frame
  glClear(GL_COLOR_BUFFER_BIT or GL_DEPTH_BUFFER_BIT) # Clear color and depth buffers
  glLoadIdentity()
  gluLookAt(bank*zoom, 0.1*zoom, 0.4*zoom, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
  mould()

proc
  dskCollide(s: var Sphere) =
  if s.position.z - s.radius <= 0:
    var
      d = s.position.z - s.radius
      n = s.velocity
    n.normalize()
    s.position.z = s.radius - d/2  # move sphere out of disk 
    s.momentum.z *= -COR  # restitution
    s.force += n*s.force

proc
  cylCollide(s: Sphere) =
  discard

proc
  spheresCollide(s1, s2: Sphere): float =
  var
    d2 = sqrLen(s2.position-s1.position)
    r2 = (s1.radius+s2.radius)
  r2 *= r2
  if d2<=r2:
    echo "sphere collide"
    return sqrt(d)
  else:
    return 0.0

proc
  sphCollide(s: var Sphere) =
  for sph in Spheres.mitems:
    if s.id != sph.id:
      if spheresCollide(s, sph):
        var n = s.velocity-sph.velocity
        n.normalize()
        s.force += n*s.force
        #sph.force += n*sph.force

proc collisions(s: var Sphere) =
  s.dskCollide()
  s.cylCollide()
  s.sphCollide()

# world
var
  s1, s2: Sphere
s1.init(0.005)
s1.position.z += 0.150
s1.quad()
s2.init(0.005)
s2.position.z += 0.17
s2.position.x += 0.003
s2.quad()
Spheres.add(s1)
Spheres.add(s2)

initGL()
reshape(width, height)

while run:
  doStuff()
  # update current time
  newTime = cpuTime()
  deltaTime = newTime - currentTime
  currentTime = newTime
  # update discrete time
  if update:  # update physics
    for s in Spheres.mitems:
      collisions(s)
      # echo "position: ", s.position.z
      # echo "force: ", s.force.z
      # echo "momentum: ", s.momentum.z
      s.update(total_time, dt)
      s.render()
    total_time += dt
    if total_time > 1:
      run = false
  win.glSwapWindow()
  fps.delay

destroy win
