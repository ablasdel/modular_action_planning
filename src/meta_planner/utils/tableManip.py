
import random
from examples import setupTableEnv

def reset_env(env, robot, includePlate=True, includeBowls=True, N=8):
    xr = (.4, .8)
    yr = (.6, .9)

    table = env.GetKinBody('table')

    origToPlace = N
    toPlace = origToPlace
    for j in xrange(100):
        items = getItems(env)
        [ env.RemoveKinBody(item) for item in items ]
        placedPlate = not includePlate
        placedBowl = not includeBowls
        for i in xrange(origToPlace*5):
            if toPlace > 0:
                x = random.random()*(xr[1] - xr[0])+xr[0]
                y = random.random()*(yr[1] - yr[0])+yr[0]
                r = random.random()
                if r < .4:
                    item = setupTableEnv.place_glass_on_table(env, table, x, y)
                elif r < .8 and x > .5 and includeBowls:
                    item = setupTableEnv.place_bowl_on_table(env, table, x, y)
                    if not env.CheckCollision(item):
                        placedBowl = True
                elif y > .75 and x > .6 and includePlate:
                    item = setupTableEnv.place_plate_on_table(env, table, x, y)
                    if not env.CheckCollision(item):
                        placedPlate = True
                else:
                    continue
                if env.CheckCollision(item):
                    env.RemoveKinBody(item)
                else:
                    toPlace -= 1
        if toPlace == 0 and (not includePlate or placedPlate) and placedBowl:
            break
        else:
            toPlace = origToPlace

def getItems(env):
    items = []
    items += [ b for b in env.GetBodies() if b.GetName().startswith('glass') ]
    items += [ b for b in env.GetBodies() if b.GetName().startswith('bowl') ]
    items += [ b for b in env.GetBodies() if b.GetName().startswith('plate') ]
    return items

