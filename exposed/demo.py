import slvs

# Taken from slvs.h
SLVS_RESULT_OKAY=0
SLVS_RESULT_INCONSISTENT=1
SLVS_RESULT_DIDNT_CONVERGE=2
SLVS_RESULT_TOO_MANY_UNKNOWNS=3

sys = slvs.System()

g = 1
# First, we create our workplane. Its origin corresponds to the origin
# of our base frame (x y z) = (0 0 0)
sys.addParam(slvs.makeParam(1, g, 0))
sys.addParam(slvs.makeParam(2, g, 0))
sys.addParam(slvs.makeParam(3, g, 0))
sys.addEntity(slvs.makePoint3d(101, g, 1, 2, 3))
#print(p.__dict__)
#sys.addPoint3d()

# and it is parallel to the xy plane, so it has basis vectors (1 0 0)
# and (0 1 0).
q = slvs.makeQuaternion(1, 0, 0, 0, 1, 0)

sys.addParam(slvs.makeParam(4, 1, q[0]))
sys.addParam(slvs.makeParam(5, 1, q[1]))
sys.addParam(slvs.makeParam(6, 1, q[2]))
sys.addParam(slvs.makeParam(7, 1, q[3]))
sys.addEntity(slvs.makeNormal3d(102,1, 4, 5, 6, 7))
sys.addEntity(slvs.makeWorkplane(200, 1, 101, 102))

# Now create a second group. We'll solve group 2, while leaving group 1
# constant; so the workplane that we've created will be locked down,
# and the solver can't move it. */
g = 2
# These points are represented by their coordinates (u v) within the
# workplane, so they need only two parameters each. 
sys.addParam(slvs.makeParam(11, g, 10))
sys.addParam(slvs.makeParam(12, g, 20))
sys.addEntity(slvs.makePoint2d(301, g, 200, 11, 12))

sys.addParam(slvs.makeParam(13, g, 20))
sys.addParam(slvs.makeParam(14, g, 10))
sys.addEntity(slvs.makePoint2d(302, g, 200, 13, 14))

# And we create a line segment with those endpoints.
sys.addEntity(slvs.makeLineSegment(400, g, 200, 301, 302))

# Now three more points
sys.addParam(slvs.makeParam(15, g, 100))
sys.addParam(slvs.makeParam(16, g, 120))
sys.addEntity(slvs.makePoint2d(303, g, 200, 15, 16))

sys.addParam(slvs.makeParam(17, g, 120))
sys.addParam(slvs.makeParam(18, g, 110))
sys.addEntity(slvs.makePoint2d(304, g, 200, 17, 18))

sys.addParam(slvs.makeParam(19, g, 115))
sys.addParam(slvs.makeParam(20, g, 115))
sys.addEntity(slvs.makePoint2d(305, g, 200, 19, 20))

# And arc, centered at point 303, starting at point 304, ending at
# point 305.
sys.addEntity(slvs.makeArcOfCircle(401, g, 200, 102, 303, 304, 305))

# Now one more point, and a distance
sys.addParam(slvs.makeParam(21, g, 200))
sys.addParam(slvs.makeParam(22, g, 200))
sys.addEntity(slvs.makePoint2d(306, g, 200, 21, 22))

sys.addParam(slvs.makeParam(23, g, 30))
sys.addEntity(slvs.makeDistance(307, g, 200, 23))

# And a complete circle, centered at point 306 with radius equal to
# distance 307. The normal is 102, the same as our workplane.
sys.addEntity(slvs.makeCircle(402, g, 200, 306, 102, 307))

#
#
#

#  The length of our line segment is 30.0 units.
sys.addConstraint(slvs.makeConstraint(1, g, slvs.SLVS_C_PT_PT_DISTANCE, 200, 30, 301, 302, 0, 0))

# And the distance from our line segment to the origin is 10.0 units.
sys.addConstraint(slvs.makeConstraint(2, g, slvs.SLVS_C_PT_LINE_DISTANCE, 200, 10, 101, 0, 400, 0))

#  And the line segment is vertical. 
sys.addConstraint(slvs.makeConstraint(3, g, slvs.SLVS_C_VERTICAL, 200, 0, 0, 0, 400, 0))

# And the distance from one endpoint to the origin is 15.0 units.
sys.addConstraint(slvs.makeConstraint(4, g, slvs.SLVS_C_PT_PT_DISTANCE, 200, 15.0, 301, 101, 0, 0))

# And same for the other endpoint; so if you add this constraint then
# the sketch is overconstrained and will signal an error.
#sys.addConstraint(slvs.makeConstraint(5, g, slvs.SLVS_C_PT_PT_DISTANCE, 200, 18, 302, 101, 0, 0))

# The arc and the circle have equal radius.
sys.addConstraint(slvs.makeConstraint(6, g, slvs.SLVS_C_EQUAL_RADIUS, 200, 0, 0, 0, 401, 402))

# The arc has radius 17.0 units.
sys.addConstraint(slvs.makeConstraint(7, g, slvs.SLVS_C_DIAMETER, 200, 17.0*2, 0, 0, 401, 0))

# If the solver fails, then ask it to report which constraints caused
# the problem.

result = sys.solve(g, True)

if result == SLVS_RESULT_OKAY:
    print("solved okay")

    print("sys.GroupHandle=%d" % sys.GroupHandle)
    print("sys.ParamHandle=%d" % sys.ParamHandle)
    print("sys.EntityHandle=%d" % sys.EntityHandle)
    print("sys.ConstraintHandle=%d" % sys.ConstraintHandle)
    
    print("line from (%.3f %.3f) to (%.3f %.3f)" % (sys.getParam(11).val, sys.getParam(12).val, sys.getParam(13).val, sys.getParam(14).val))
    print("arc center (%.3f %.3f) start (%.3f %.3f) finish (%.3f %.3f)" %
        (sys.getParam(15).val, sys.getParam(16).val, sys.getParam(17).val, 
         sys.getParam(18).val, sys.getParam(19).val, sys.getParam(20).val))
    print("circle center (%.3f %.3f) radius %.3f" % 
        (sys.getParam(21).val, sys.getParam(22).val, sys.getParam(23).val))

    print("DOF=%d" % sys.Dof)

    #for i in range(0, 24):
    #    try:
    #        print("%d: %.3f" % (i, sys.getParam(i).val))
    #    except:
    #        pass
else:
    print("solve failed: problematic constraints are:")
    for elem in sys.Failed:
        print(elem)
    if result == SLVS_RESULT_INCONSISTENT:
        print("system inconsistent")
    elif result == SLVS_RESULT_DIDNT_CONVERGE:
        print("system nonconvergent")
    else:
        print("too many unknowns")
