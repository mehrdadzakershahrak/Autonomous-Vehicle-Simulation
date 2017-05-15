from numpy.linalg import lstsq
from numpy import ones, vstack, isclose

def get_m_c(points):
    x_coords, y_coords = zip(*points)
    A = vstack( [x_coords, ones(len(x_coords))] ).T
    m, c = lstsq( A, y_coords )[0]
    return (m, c)

def is_covered_x(((x1, y1), (x2, y2)), ((x3, y3), (x4, y4))):
    if min(x1, x2) <= min(x3, x4) and max(x3, x4) <= max(x1, x2):
        return True
    elif min(x1, x2) <= min(x3, x4) and max(x1, x2) <= max(x3, x4):
        return True
    elif min(x3, x4) <= min(x1, x2) and max(x1, x2) <= max(x3, x4):
        return True
    elif min(x3, x4) <= min(x1, x2) and max(x3, x4) <= max(x1, x2):
        return True
    elif min(x1, x2) <= min(x3, x4) and max(x1, x2) <= max(x3, x4):
        return True
    else:
        return False

def is_covered_y(((x1, y1), (x2, y2)), ((x3, y3), (x4, y4))):
    if min(y1, y2) <= min(y3, y4) and max(y3, y4) <= max(y1, y2):
        return True
    elif min(y1, y2) <= min(y3, y4) and max(y1, y2) <= max(y3, y4):
        return True
    elif min(y3, y4) <= min(y1, y2) and max(y1, y2) <= max(y3, y4):
        return True
    elif min(y3, y4) <= min(y1, y2) and max(y3, y4) <= max(y1, y2):
        return True
    elif min(y1, y2) <= min(y3, y4) and max(y1, y2) <= max(y3, y4):
        return True
    else:
        return False

def hh(points1, points2):
    (((x1, y1), (x2, y2)), ((x3, y3), (x4, y4))) = (points1, points2)
    if y1 == y3:
        return is_covered_x(points1, points2)
    return False

def hhp(points1, points2):
    (((x1, y1), (x2, y2)), ((x3, y3), (x4, y4))) = (points1, points2)
    assert(y1 == y3)
    assert(is_covered_x(points1, points2))

    if min(x1, x2) <= min(x3, x4) and max(x3, x4) <= max(x1, x2):
        #return (min(x3, x4), y3)
        return ((x3 + x4) / 2.0, y3)
    elif min(x1, x2) <= min(x3, x4) and max(x1, x2) <= max(x3, x4):
        #return (min(x3, x4), y3)
        return ((min(x3, x4) + max(x1, x2)) / 2.0, y3)
    elif min(x3, x4) <= min(x1, x2) and max(x1, x2) <= max(x3, x4):
        #return (min(x1, x2), y3)
        return ((x1 + x2) / 2.0, y3)
    elif min(x3, x4) <= min(x1, x2) and max(x3, x4) <= max(x1, x2):
        #return (min(x1, x2), y3)
        return ((min(x1, x2) + max(x3, x4)) / 2.0, y3)
    elif min(x1, x2) <= min(x3, x4) and max(x3, x4) <= max(x1, x2):
        #return (min(x3, x4), y3)
        return ((x3 + x4) / 2.0, y3)
    else:
        raise BaseException, "Shouldn't happen if is_covered_x!"

def hv(points1, points2):
    (((x1, y1), (x2, y2)), ((x3, y3), (x4, y4))) = (points1, points2)
    if min(x1, x2) <= x3 and x3 <= max(x1, x2):
        if min(y3, y4) <= y1 and y1 <= max(y3, y4):
            return True
    return False

def hvp(points1, points2):
    (((x1, y1), (x2, y2)), ((x3, y3), (x4, y4))) = (points1, points2)
    assert(hv(points1, points2))

    return (x3, y1) # here x3 == x4, y1 == y2 since horizontal, vertical, resp.

def vh(points1, points2):
    (((x1, y1), (x2, y2)), ((x3, y3), (x4, y4))) = (points1, points2)
    if min(y1, y2) <= y3 and y3 <= max(y1, y2):
        if min(x3, x4) <= x1 and x1 <= max(x3, x4):
            return True
    return False

def vhp(points1, points2):
    (((x1, y1), (x2, y2)), ((x3, y3), (x4, y4))) = (points1, points2)
    assert(vh(points1, points2))

    return (x1, y3) # here x1 == x2, y3 == y4 since vertical, horizontal, resp.

def vv(points1, points2):
    (((x1, y1), (x2, y2)), ((x3, y3), (x4, y4))) = (points1, points2)
    if x1 == x3:
        return is_covered_y(points1, points2)
    return False

def vvp(points1, points2):
    (((x1, y1), (x2, y2)), ((x3, y3), (x4, y4))) = (points1, points2)
    assert(x1 == x3)
    assert(is_covered_y(points1, points2))

    if min(y1, y2) <= min(y3, y4) and max(y3, y4) <= max(y1, y2):
        #return (min(y3, y4), x3)
        return ((y3 + y4) / 2.0, x3)
    elif min(y1, y2) <= min(y3, y4) and max(y1, y2) <= max(y3, y4):
        #return (min(y3, y4), x3)
        return ((min(y3, y4) + max(y1, y2)) / 2.0, x3)
    elif min(y3, y4) <= min(y1, y2) and max(y1, y2) <= max(y3, y4):
        #return (min(y1, y2), x3)
        return ((y1 + y2) / 2.0, x3)
    elif min(y3, y4) <= min(y1, y2) and max(y3, y4) <= max(y1, y2):
        #return (min(y1, y2), x3)
        return ((min(y1, y2) + max(y3, y4)) / 2.0, x3)
    elif min(y1, y2) <= min(y3, y4) and max(y1, y2) <= max(y3, y4):
        #return (min(y3, y4), x3)
        return ((min(y3, y4) + max(y1, y2)) / 2.0, x3)
    else:
        raise BaseException, "Shouldn't happen if is_covered_y!"

def hx(points1, points2):
    (((x1, y1), (x2, y2)), ((x3, y3), (x4, y4))) = (points1, points2)
    (m, c) = get_m_c(points2)
    x5 = (y1 - c) / m
    if min(x1, x2) <= x5 and x5 <= max(x1, x2):
        if min(x3, x4) <= x5 and x5 <= max(x3, x4):
            return True
    return False

def hxp(points1, points2):
    (((x1, y1), (x2, y2)), ((x3, y3), (x4, y4))) = (points1, points2)
    (m, c) = get_m_c(points2)
    x5 = (y1 - c) / m   # here y1 == y2 since it is horizontal
    assert(hx(points1, points2))

    return (x5, y1)

def xh(points1, points2):
    (((x1, y1), (x2, y2)), ((x3, y3), (x4, y4))) = (points1, points2)
    (m, c) = get_m_c(points1)
    x5 = (y3 - c) / m
    if min(x1, x2) <= x5 and x5 <= max(x1, x2):
        if min(x3, x4) <= x5 and x5 <= max(x3, x4):
            return True
    return False

def xhp(points1, points2):
    (((x1, y1), (x2, y2)), ((x3, y3), (x4, y4))) = (points1, points2)
    (m, c) = get_m_c(points1)
    x5 = (y3 - c) / m   # here y3 == y4 since it is horizontal
    assert(xh(points1, points2))

    return (x5, y3)

def vx(points1, points2):
    (((x1, y1), (x2, y2)), ((x3, y3), (x4, y4))) = (points1, points2)
    (m, c) = get_m_c(points2)
    y5 = x1 * m + c     # here x1 == x2 since it is vertical
    if min(y1, y2) <= y5 and y5 <= max(y1, y2):
        if min(x3, x4) <= x1 and x1 <= max(x3, x4):
            return True
    return False

    # if min(y1, y2) <= y3 and y3 <= max(y1, y2):
    #     if min(x3, x4) <= x1 and x1 <= max(x3, x4):
    #         return True
    # return False

def vxp(points1, points2):
    (((x1, y1), (x2, y2)), ((x3, y3), (x4, y4))) = (points1, points2)
    (m, c) = get_m_c(points2)
    y5 = x1 * m + c     # here x1 == x2 since it is vertical
    assert(vx(points1, points2))

    return (x1, y5)

def xv(points1, points2):
    (((x1, y1), (x2, y2)), ((x3, y3), (x4, y4))) = (points1, points2)
    (m, c) = get_m_c(points1)
    y5 = x3 * m + c
    if min(y1, y2) <= y5 and y5 <= max(y1, y2):
        if min(y3, y4) <= y5 and y5 <= max(y3, y4):
            return True
    return False

def xvp(points1, points2):
    (((x1, y1), (x2, y2)), ((x3, y3), (x4, y4))) = (points1, points2)
    (m, c) = get_m_c(points1)
    y5 = x3 * m + c
    assert(xv(points1, points2))

    return (x3, y5) # Here x3 == x4 since it is vertical

def xx(points1, points2):
    (((x1, y1), (x2, y2)), ((x3, y3), (x4, y4))) = (points1, points2)
    L1 = line((x1, y1), (x2, y2))
    L2 = line((x3, y3), (x4, y4))
    R = intersection(L1, L2)
    if R:
        (x5, y5) = R
        if min(x1, x2) <= x5 and x5 <= max(x1, x2):
            if min(y1, y2) <= y5 and y5 <= max(y1, y2):
                if min(x3, x4) <= x5 and x5 <= max(x3, x4):
                    if min(y3, y4) <= y5 and y5 <= max(y3, y4):
                        return True
        return False
    else:
        return False

def xxp(points1, points2):
    (((x1, y1), (x2, y2)), ((x3, y3), (x4, y4))) = (points1, points2)
    L1 = line((x1, y1), (x2, y2))
    L2 = line((x3, y3), (x4, y4))
    R = intersection(L1, L2)
    assert(xx(points1, points2))
    (x5, y5) = R
    return (x5, y5)

def line((x1, y1), (x2, y2)):
    A = y1 - y2
    B = x2 - x1
    C = x1 * y2 - x2 * y1
    return A, B, -C

def intersection(L1, L2):
    D = L1[0] * L2[1] - L1[1] * L2[0]
    Dx = L1[2] * L2[1] - L1[1] * L2[2]
    Dy = L1[0] * L2[2] - L1[2] * L2[0]
    if D != 0:
        x = Dx / D
        y = Dy / D
        return x, y
    else:
        return False

def is_horizontal(points):
    (x1, y1), (x2, y2) = points
    if x1 != x2 and y1 == y2:
        return True
    else:
        return False

def is_vertical(points):
    (x1, y1), (x2, y2) = points
    if x1 == x2 and y1 != y2:
        return True
    else:
        return False

def is_intersected(ps1, ps2):
    if is_horizontal(ps1):
        if is_horizontal(ps2):
            return hh(ps1, ps2)
        elif is_vertical(ps2):
            return hv(ps1, ps2)
        else:
            return hx(ps1, ps2)
    elif is_vertical(ps1):
        if is_vertical(ps2):
            return vv(ps1, ps2)
        elif is_horizontal(ps2):
            return vh(ps1, ps2)
        else:
            return vx(ps1, ps2)
    else:
        if is_horizontal(ps2):
            return xh(ps1, ps2)
        elif is_vertical(ps2):
            return xv(ps1, ps2)
        else:
            return xx(ps1, ps2)

def get_intersection(ps1, ps2):
    #if not is_intersected(ps1, ps2):
    #    msg = "two lines (%s), (%s) are not intersected" % ps1, ps2
    #    raise BaseException, msg
    assert(is_intersected(ps1, ps2))

    if is_horizontal(ps1):
        if is_horizontal(ps2):
            return hhp(ps1, ps2)
        elif is_vertical(ps2):
            return hvp(ps1, ps2)
        else:
            return hxp(ps1, ps2)
    elif is_vertical(ps1):
        if is_vertical(ps2):
            return vvp(ps1, ps2)
        elif is_horizontal(ps2):
            return vhp(ps1, ps2)
        else:
            return vxp(ps1, ps2)
    else:
        if is_horizontal(ps2):
            return xhp(ps1, ps2)
        elif is_vertical(ps2):
            return xvp(ps1, ps2)
        else:
            return xxp(ps1, ps2)

def get_midpoint(points):
    (x1, y1), (x2, y2) = points
    return ((x1 + x2) * 0.5, (y1 + y2) * 0.5)

def on_borders(ps1, ps2):
    (p1, p2) = ps2
    p3 = get_midpoint(ps2)

    #return on_border(ps1, (p1, p3)) and on_border(ps1, (p3, p2))
    i1 = is_intersected(ps1, (p1, p3))
    i2 = is_intersected(ps1, (p3, p2))
    return i1 and i2

def on_border(ps1, ps2):
    if is_horizontal(ps1):
        if is_horizontal(ps2):
            return hh(ps1, ps2)
        elif is_vertical(ps2):
            return False
        else:
            return hx(ps1, ps2)
    elif is_vertical(ps1):
        if is_vertical(ps2):
            return vv(ps1, ps2)
        elif is_horizontal(ps2):
            return False
        else:
            return vx(ps1, ps2)
    else:
        if is_horizontal(ps2):
            return xh(ps1, ps2)
        elif is_vertical(ps2):
            return xv(ps1, ps2)
        else:
            return xx(ps1, ps2)

if __name__ == "__main__":
    print is_intersected(((1, 1), (10, 1)), ((1, 2), (10, 2)))
    print is_intersected(((10, 0), (0, 10)), ((0, 0), (10, 10)))
    print is_intersected(((-5, -5), (0, 0)), ((1, 1), (10, 10)))
    
    ps1 = ((6.0, 5.0), (6.0, 495.0))
    ps2 = ((6.0, 495.0), (6.0, 5.0))
    ps3 = ((6.0, 127.5), (6.0, 5.0))
    print is_intersected(ps1, ps2)
    print is_intersected(ps1, ps3)
    
    ps4 = ((-8000, 25000), (-5290.945, 12198.925))
    ps5 = ((-7000, 19000), (-5202.545, 12041.925))
    
    ps6 = ((-8000, 24000), (-5290.945, 11883.745))
    ps7 = ((-7000, 19000), (-5202.545, 12041.925))
    
    print is_intersected(ps4, ps5)
    print is_intersected(ps6, ps7)

    ps1 = ((395.0, 75.0), (346.25, 75.0))
    ps2 = ((297.5, 75.0), (297.5, 75.0))
    ps3 = ((5.0, 75.0), (395.0, 75.0))

    print is_intersected(ps1, ps3)
    print is_intersected(ps2, ps3)

    ps4 = ((395.0, 75.0), (297.5, 75.0))
    print on_borders(ps3, ps4)
