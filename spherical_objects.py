import math
from typing import List
# Lens constants assuming a 1080p image
f = 714.285714
center = [960, 540]
D = 1.082984  # For image-1, switch to 0.871413 for image-2

#####################
'''
tita = asen (OP/HIP) == acos(ADY/HIP)
tita = atan(OP/ADY)

------------------------------------------------------
cartesiano a esferico
rho**2 = x**2  y**2 + z**2
tan tita = y/x -> tita = atan(y/x)
phi = acos (z/rho)
Si cos(phi) =0  -> phi = (2k+1)*pi/2

-------------------------------------------------------
esferico a cartesiano
x = rho * sin(phi) * cos(tita)
y = rho * sin(phi) * sin(tita)
z = rho * cos(phi)

'''

####################
def cartesian2sphere(pt):
    x = (pt[0] - center[0]) / f
    y = (pt[1] - center[1]) / f

    r = math.sqrt(x*x + y*y)
    if r != 0:
        x /= r
        y /= r
    r *= D
    sin_theta = math.sin(r)
    x *= sin_theta
    y *= sin_theta
    z = math.cos(r)

    return [x, y, z]


def sphere2cartesian(pt):
    r = math.acos(pt[2])
    r /= D
    if pt[2] != 1:
        r /= math.sqrt(1 - pt[2] * pt[2])
    x = r * pt[0] * f + center[0]
    y = r * pt[1] * f + center[1]
    return [x, y]

## BUG return list is not of ints...
def convert_point(point: List[int]) -> List[int]:
    """Convert single points between Cartesian and spherical coordinate systems"""
    if len(point) == 2:
        return cartesian2sphere(point)
    elif len(point) == 3:
        return sphere2cartesian(point)
    else:
        raise ValueError(f'Expected point to be 2 or 3D, got {len(point)} dimensions')


class CartesianBbox:

    def __init__(self, points: List[int], fmt: str):
        assert fmt in ['xyxy', 'xywh', 'cxcywh'], 'Invalid bbox format'
        assert len(points) == 4, 'Cartesian bbox must have 4 values'
        self.points = points
        self.fmt = fmt

####################################################################################################
class SphericalBbox:

    def __init__(self, points: List[List[int]]):
        self.points = points
        assert len(points) == 4, 'Spherical bbox must have 4 values'
        for point in points:
            assert len(point) == 3, 'Spherical point must have 3 values'

def bbox_to_spherical(cartesian: CartesianBbox) -> SphericalBbox:
    points = cartesian.points
    # Convert to 4 points
    if cartesian.fmt == 'xyxy':
        x_1,y_1 = points[0:2]
        width,height = points[2]-points[0], points[3]- points[1]
    elif cartesian.fmt == 'xywh':
        x_1,y_1,width,height = points
    else:
        width,height = points[2:]
        x_1 = points[0] - int(width/2)
        y_1 = points[1] - int(height/2)

    p_0 = convert_point([x_1, y_1])
    p_1 = convert_point([x_1+width, y_1])
    p_2 = convert_point([x_1+width, y_1+height])
    p_3 = convert_point([x_1, y_1+height])

    return SphericalBbox([p_0,p_1,p_2,p_3])


class CartesianPolygon:

    def __init__(self, points: List[List[int]]):
        self.points=points
        for point in points:
            assert len(point) == 2, 'cartesian polygon point must have 2 values'
        # Enable or disable practical limit uncommenting or commenting next line
        # assert len(points) <=20, 'Practical limit..'
        
        # Check Polygon points for ??
        # For a BBOX the Polygon should be concave or convex, but not complex (crossing lines)
        # regular.. all side equal and sum( angles)==180º.
        #  Probaly, i can check if a polygon is regular and angles ==90 is a square, and reduce to only four points
        # similar for a rectangoe ( irregular polygon )
        # this is similar to the countour simples/complex and approxymate polydp from opencv
        # tal vez convertir a un cnt (opencv) y ahi hay un par de funciones para chequar cosas


class SphericalPolygon:

    def __init__(self, points:List[List[int]]):
        self.points = points
        for point in points:
            assert len(point) == 3, 'Spherical polygon point must have 3 values'
        # Enable or disable practical limit uncommenting or commenting next line
        # assert len(points) <=20, 'Practical limit..'



def polygon_to_spherical(cartesian: CartesianPolygon) -> SphericalPolygon:
    sphere_points = []
    for point in cartesian.points:
        tmp = convert_point(point)
        sphere_points.append(tmp)
    return SphericalPolygon(sphere_points)


if __name__ == "__main__":

    # Test convertion function
    cartesian_point = [100,100]
    spherical_point = convert_point(cartesian_point)
    cartesian_point2 = convert_point(spherical_point)

    # Test CartesianBox and SphericalBox
    cartesian_box = CartesianBbox([2,7, 3,3], fmt='xywh')
    spherical_box = bbox_to_spherical(cartesian_box)

    # Test CartesianPolygon and SphericalPolygon

    # create a square with lot of points
    def box():
        _x=100
        _y=100
        last_y=1000
        top_horizontal_line = []
        for x in range(100,1920, 200):
            top_horizontal_line.append([x,_y])
        last_x=top_horizontal_line[-1][0]

        left_vertical_line=[]
        for y in range(200, 920, 100):
            left_vertical_line.append([_x,y])
        # last_y=left_vertical_line[-1][1]

        right_vertical_line = [[last_x,p[1]] for p in left_vertical_line]
        bottom_horizontal_line = [[p[0],last_y] for p in reversed(top_horizontal_line)]

        left_vertical_line = list(reversed(left_vertical_line))

        box_points = []
        box_points.extend(top_horizontal_line)
        box_points.extend(right_vertical_line)
        box_points.extend(bottom_horizontal_line)
        box_points.extend(left_vertical_line)

        return box_points
    box_points = box()
    cartesian_polygon = CartesianPolygon(box_points)
    spherical_polygon = polygon_to_spherical(cartesian_polygon)
