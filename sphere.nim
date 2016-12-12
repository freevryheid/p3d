import
  math,
  basic3d,
  quaternion,
  opengl,
  glu

type
  Sphere* = object
    # primary physics state
    position*: Vector3d             # the position of the sphere center of mass in world coordinates (meters).
    momentum*: Vector3d             # the momentum of the sphere in kilogram meters per second.
    orientation*: Quaternion        # the orientation of the sphere represented by a unit quaternion.
    angularMomentum*: Vector3d      # angular momentum vector.
    # seconadary state
    velocity*: Vector3d             # velocity in meters per second (calculated from momentum).
    spin*: Quaternion               # quaternion rate of change in orientation.
    angularVelocity*: Vector3d      # angular velocity (calculated from angularMomentum).
    bodyToWorld*: Matrix3d          # body to world coordinates matrix.
    worldToBody*: Matrix3d          # world to body coordinates matrix.
    # constant state
    radius*: float                  # radius of the sphere in meters.
    mass*: float                    # mass of the sphere in kilograms.
    inverseMass*: float             # inverse of the mass used to convert momentum to velocity.
    inertiaTensor*: float           # inertia tensor of the sphere (simplified to a single value due to the mass properties of a sphere).
    inverseInertiaTensor*: float    # inverse inertia tensor used to convert angular momentum to angular velocity.
    # forces
    force*: Vector3d
    torque*: Vector3d
    # other
    quad: GLUquadric
    id*: int

  # Derivative values for primary state.
  # This structure stores all derivative values for primary state in Sphere.
  # For example velocity is the derivative of position, force is the derivative
  # of momentum etc. Storing all derivatives in this structure makes it easy
  # to implement the RK4 integrator cleanly because it needs to calculate the
  # and store derivative values at several points each timestep.
  Derivative* = object
    velocity: Vector3d              # velocity is the derivative of position.
    force: Vector3d                 # force in the derivative of momentum.
    spin: Quaternion                # spin is the derivative of the orientation quaternion.
    torque: Vector3d                # torque is the derivative of angular momentum.

var
   counter = 0

proc
  recalculate(s: var Sphere) =
  # recalculate secondary state values from primary values.
  s.velocity = s.momentum * s.inverseMass
  s.angularVelocity = s.angularMomentum * s.inverseInertiaTensor
  s.orientation = s.orientation.normalize()
  s.spin = 0.5 * quaternion(0, s.angularVelocity.x, s.angularVelocity.y, s.angularVelocity.z) * s.orientation
  #var translation: Matrix3d = s.position.move()
  #s.bodyToWorld = translation * s.orientation.matrix()
  #s.worldToBody = s.bodyToWorld.inverse()

proc
  init*(s: var Sphere, radius: float) =
  s.radius = radius
  var V=4.0/3.0*PI*s.radius*s.radius*s.radius
  s.mass = 2650.0*V
  s.inverseMass = 1.0 / s.mass
  s.force = vector3d(0.0,0.0,0.0)
  s.torque = vector3d(0.0,0.0,0.0)
  s.position = vector3d(0.0,0.0,0.0)
  s.momentum = vector3d(0.0,0.0,0.0)
  s.orientation = IDQUAT
  s.angularMomentum = vector3d(0.0,0.0,0.0)
  s.inertiaTensor = 2.0/5.0 * s.mass * s.radius*s.radius
  s.inverseInertiaTensor = 1.0 / s.inertiaTensor
  s.recalculate()
  #s.quad = gluNewQuadric()
  #gluQuadricDrawStyle(s.quad, GLEnum(GLU_LINE))

proc
  quad*(s: var Sphere) =
  s.quad = gluNewQuadric()
  gluQuadricDrawStyle(s.quad, GLEnum(GLU_LINE))
  s.id = counter
  inc counter
  
proc
  render*(s: Sphere) =
  glPushMatrix()
  glTranslatef(s.position.x, s.position.y, s.position.z)
  var aa: AA = angleAxis(s.orientation)
  glRotatef(aa.angle/PI*180.0, aa.axis.x, aa.axis.y, aa.axis.z)
  gluSphere(s.quad, s.radius, 8, 8)
  glPopMatrix()

# Calculate force and torque for physics state at time t.
# Due to the way that the RK4 integrator works we need to calculate
# force implicitly from state rather than explictly applying forces
# to the rigid body once per update. This is because the RK4 achieves
# its accuracy by detecting curvature in derivative values over the
# timestep so we need our force values to supply the curvature.
proc
  forces(s: var Sphere, t: float) =
  #discard
  s.force.z = -s.mass*9.81  # gravity force

# Evaluate all derivative values for the physics state at time t.
# @param state the physics state of the sphere.
proc
  evaluate(s: var Sphere, t: float): Derivative =
  result.velocity = s.velocity
  result.spin = s.spin
  forces(s, t)
  result.force = s.force
  result.torque = s.torque

# Evaluate derivative values for the physics state at future time t+dt
# using the specified set of derivatives to advance dt seconds from the
# specified physics state.
proc
  evaluate(s: var Sphere, t, dt: float, derivative: Derivative): Derivative =
  var ss: Sphere
  ss.init(s.radius)
  ss.position = s.position + derivative.velocity * dt
  ss.momentum = s.momentum + derivative.force * dt
  ss.orientation = s.orientation + derivative.spin * dt
  ss.angularMomentum = s.angularMomentum + derivative.torque * dt
  ss.recalculate()

  # s.position += derivative.velocity * dt
  # s.momentum += derivative.force * dt
  # s.orientation += derivative.spin * dt
  # s.angularMomentum += derivative.torque * dt
  # s.recalculate()

  result.velocity = ss.velocity
  result.spin = ss.spin
  forces(s, t+dt)
  result.force = s.force
  result.torque = s.torque

# Integrate physics state forward by dt seconds.
# Uses an RK4 integrator to numerically integrate with error O(5).
proc
  integrate(s: var Sphere, t, dt: float) =
  var
    a: Derivative = evaluate(s, t)
    b: Derivative = evaluate(s, t, dt*0.5, a)
    c: Derivative = evaluate(s, t, dt*0.5, b)
    d: Derivative = evaluate(s, t, dt, c)
  s.position += 1.0/6.0 * dt * (a.velocity + 2.0*(b.velocity + c.velocity) + d.velocity)
  s.momentum += 1.0/6.0 * dt * (a.force + 2.0*(b.force + c.force) + d.force)
  s.orientation += 1.0/6.0 * dt * (a.spin + 2.0*(b.spin + c.spin) + d.spin)
  s.angularMomentum += 1.0/6.0 * dt * (a.torque + 2.0*(b.torque + c.torque) + d.torque)
  s.recalculate()

# update physics state.
proc
  update*(s: var Sphere, t, dt: float) =
  integrate(s, t, dt)
