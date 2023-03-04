import Rhino.Geometry as rg
from copy import deepcopy
import math
import perlin
import random


class Wall(object):
    def __init__(self, curve, layer_num, gap, length, width, height):
        self.curve = curve
        self.layer_num = layer_num
        self.gap = gap
        self.length = length
        self.width = width
        self.height = height
        self.get_t_points()
        self.get_plane()
        self.deformation()
        self.brick()
        
    def get_t_points(self): 
        self.t_values = []
        self.t_points = []
        self.crvs = []       #layers
        self.id=[]
        for i in range(self.layer_num):
            crv = deepcopy(self.curve)
            tran = rg.Transform.Translation(rg.Vector3d(0, 0, height*i))
            crv.Transform(tran)
            if i%2 == 0:
                point = crv.PointAtLength(self.length/2) #point
                closest = crv.ClosestPoint(point) #returns boolean + parameter at closest point
                param = closest[1] #parameter for trimming curve
                crv= rg.Curve.Trim(crv, param,crv.Domain[1]) #parameter
            self.crvs.append(crv)
            a = crv.DivideByLength(self.length+self.gap,False)
            
            for j in a:
                self.t_points.append(crv.PointAt(j))
                self.t_values.extend(a)
                self.id.append(i)  #to know each point on which layer


    def get_plane(self):
        self.planes = []
        # zip in the list with same/shortest length
        for value, point,id in zip(self.t_values, self.t_points, self.id):
            #rg.Curve.TangentAt(  TangentAt(self: Curve, t: float) -> Vector3d
            #rg.Plane( Plane(origin: Point3d, normal: Vector3d)
            plane = rg.Plane(point, self.crvs[id].TangentAt(value))          
            tran = rg.Transform.Rotation(math.radians(90), plane.XAxis, plane.Origin)
            plane.Transform(tran)
            if plane.ZAxis.Z <0:
                tran2 = rg.Transform.Rotation(math.radians(180), plane.XAxis, plane.Origin)
                tran3 = rg.Transform.Rotation(math.radians(180), plane.ZAxis, plane.Origin)
                trans = tran2 * tran3
                plane.Transform(trans)
            self.planes.append(plane)
# Rotation(angleRadians: float, rotationAxis: Vector3d, rotationCenter: Point3d)

    def remap(self, value, low1, high1, low2, high2):
        new_value = low2 + (value - low1) *(high2 - low2)/ (high1-low1)
        return new_value

    def deformation(self):
        self.new_planes = []
        if type == 0:
            sn = perlin.SimplexNoise()
            for i, p in enumerate(self.planes):
                noisevalue = sn.noise3(p.Origin.X/scale, p.Origin.Y/scale, p.Origin.Z/scale)
                xshift = self.remap(noisevalue, -1, 1, 0, 0.01)
                yshift = self.remap(noisevalue, -1, 1, 0, 0.01)
                tran1 = rg.Transform.Translation(rg.Vector3d(xshift*p.Origin.X, yshift*p.Origin.Y, 0))
                angle = self.remap(noisevalue, -1, 1, 0, 0.5*math.pi)
                tran2 = rg.Transform.Rotation(angle, p.Origin)
                trans = tran1*tran2
                p.Transform(trans)
                self.new_planes.append(p)
                
        elif type == 1:
            distances = []
            for i, p in enumerate(self.planes):
                #rg.Curve.ClosestPoint(attract_curve, p.Origin[i], 50.0)  (bool,float)
                t = attract_curve.ClosestPoint(p.Origin, 50.0)[1]
                closestp = rg.Curve.PointAt(attract_curve, t)                                     
                # PointAt(self: Curve, t: float) -> Point3d
                distances.append(p.Origin.DistanceTo(closestp))
            distances.sort()
            dmin = distances[0]
            dmax = distances[-1]
            range = dmax-dmin
            for i, j in enumerate(self.planes):
                scalist = (distances[i]-dmax)/range*(max-min)
                tran2 = rg.Transform.Rotation(0.1*scalist*math.pi, j.Origin)
                j.Transform(tran2)
                self.new_planes.append(j)
                
        elif type == 2:
            distances = []
            for i, p in enumerate(self.planes):
                distances.append(p.Origin.DistanceTo(attract_point))
            distances.sort()
            dmin = distances[0]
            dmax = distances[-1]
            range = dmax-dmin
            for i, j in enumerate(self.planes):
                scalist = (distances[i]-dmax)/range*(max-min)
                tran2 = rg.Transform.Rotation(0.1*scalist*math.pi, j.Origin)
                j.Transform(tran2)
                self.new_planes.append(j)
               
        else:
            self.new_planes = self.planes

    def brick(self):
        self.bricks = []
        for p in self.new_planes:
            brick = rg.Box(p, rg.Interval(-self.width/2,self.width/2), rg.Interval(-self.length/2,self.length/2), rg.Interval(0,self.height))
            tran = rg.Transform.Rotation(angle, p.Origin)
            brick.Transform(tran)
            self.bricks.append(brick)



a = Wall(curve, layer_num, gap, length, width, height)
new_planes = a.new_planes
bricks = a.bricks
crvs = a.crvs
t_points =a.t_points
planes = a.planes
t_values = a.t_values