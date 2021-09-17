# Copyright 2021, Benoit Marchal - https://www.marchal.com

import os, re, sys, math, argparse, logging, functools, itertools

# https://stackoverflow.com/questions/3365171/calculating-the-angle-between-two-lines-without-having-to-calculate-the-slope
def angle(point0, point1, point2):
    x0, y0 = point0
    x1, y1 = point1
    x2, y2 = point2
    if x0 == x1:
        angle1 = math.pi / 2
    else:
        angle1 = math.atan((y1 - y0) / (x1 - x0))
    if x1 == x2:
        angle2 = math.pi / 2
    else:
        angle2 = math.atan((y2 - y1) / (x2 - x1))
    return math.degrees(abs(angle2 - angle1))

CLK  = "G2"
CCLK = "G3"
# https://stackoverflow.com/questions/27635188/algorithm-to-detect-left-or-right-turn-from-x-y-co-ordinates
def direction(point0, point1, point2):
    x0, y0 = point0
    x1, y1 = point1
    x2, y2 = point2
    v1x = x1 - x0
    v1y = y1 - y0
    v2x = x2 - x1
    v2y = y2 - y1
    if v1x * v2y - v1y * v2x < 0.0:
        return CLK
    else:
        return CCLK

def distance(point0, point1):
    x0, y0 = point0
    x1, y1 = point1
    return math.sqrt(((x0 - x1) ** 2) + ((y0 - y1) ** 2))

# https://math.stackexchange.com/questions/134112/find-a-point-on-a-line-segment-located-at-a-distance-d-from-one-endpoint
def shift(from0, point0, point1, length):
    x0, y0 = point0
    x1, y1 = point1
    if from0:
        xa, ya = point0
    else:
        xa, ya = point1
    d = distance(point0, point1)
    assert d != 0, "not a line"
    xd = xa + ((length * (x1 - x0)) / d)
    yd = ya + ((length * (y1 - y0)) / d)
    assert round(distance((xa, ya), (xd, yd)), 6) >= length
    return (xd, yd)
shift_start = functools.partial(shift, True)
shift_end   = functools.partial(shift, False)

def coord(pattern, code, default):
    m = pattern.search(code)
    if m is None:
        return default
    else:
        return float(m.group(1))
pos_x = functools.partial(coord, re.compile(r"X([\d\.\-]+)"))
pos_y = functools.partial(coord, re.compile(r"Y([\d\.\-]+)"))
pos_z = functools.partial(coord, re.compile(r"Z([\d\.\-]+)"))

def gxy(command, origin, destination, **extras):
    CIRCULAR = ("G2", "G3")
    assert "center" in extras if command in CIRCULAR else True
    xo, yo = origin
    xd, yd = destination
    if origin != destination:    #outputs only if we have a move
        yield command
        if xo != xd:
            yield "X{:.4g}".format(xd)
        if yo != yd:
            yield "Y{:.4g}".format(yd)
        if command in CIRCULAR:
            xc, yc = extras["center"]
            yield "I{:.4g}J{:.4g}".format(xc - xo, yc - yo)
        yield "\n"

def gz(z):
    yield "G0Z{:.4g}".format(z)
    yield "\n"

def preprocess(block):
    # remove EOL (to ensure consistency in final output) and ignore comments
    block = block.rstrip()
    found = preprocess.__COMMENT.search(block)
    if found and found.group(1):
        return (block, found.group(1))
    else:
        return (block, block)
preprocess.__COMMENT = re.compile(r"(\s*);\s*")

class motion():
    def __init__(self, radius, retract, safe_retract, sharp_angle, select = None):
        assert 0 <= sharp_angle <= 90
        self.coordinates = [(0.0, 0.0)]
        self.z = 0.0
        self.skipping = False
        self.radius = radius
        self.retract = retract
        self.safe_retract = safe_retract
        self.angle_min = sharp_angle
        self.angle_max = 180 - sharp_angle
        self.CRITICAL_PARAMETER = re.compile(r"[SF][\d\.\-]+")
        self.BLOCK = 99
        if select:
            self.__call__ = lambda *args: filter(select, self.__process(*args))
        else:
            self.__call__ = self.__process

    def __sharp_angle(self):
        assert len(self.coordinates) > 2
        return self.angle_max > angle(self.coordinates[-3],
                                      self.coordinates[-2],
                                      self.coordinates[-1]) > self.angle_min

    def __process(self, speed, code):
        xp, yp = self.coordinates[-1]
        self.coordinates.append((pos_x(code, xp), pos_y(code, yp)))
        self.z = pos_z(code, self.z)
        if self.coordinates[-2] == self.coordinates[-1]:
            self.coordinates.pop(0)
            yield self.BLOCK
        elif speed == 0:
            while len(self.coordinates) > 1:
                self.coordinates.pop(0)
            self.skipping = False
            logging.debug("raising blade: %s", code)
            for op in itertools.chain(gz(self.z + self.safe_retract),
                                      itertools.repeat(self.BLOCK, 1),
                                      gz(self.z)):
                yield op
        elif speed == 1:
            length = distance(self.coordinates[-2], self.coordinates[-1])
            self.skipping = not self.skipping and length <= self.radius and \
                            len(self.coordinates) > 2
            if not self.skipping and len(self.coordinates) > 2 and self.__sharp_angle():
                if length < self.radius:
                    # the knife cannot do short turns; if there's a sharp turn with too
                    # small a move, skip the move until the knife will have travel enough
                    self.skipping = True
                else:
                    # sharp turn, rotate around the tip of the blade
                    travel = shift_end(self.coordinates[-3], self.coordinates[-2],
                                       self.radius)
                    swivel = shift_start(self.coordinates[-2], self.coordinates[-1],
                                         self.radius)
                    turn = direction(self.coordinates[-3],
                                     self.coordinates[-2],
                                     self.coordinates[-1])
                    logging.debug("rotating blade: %s", code)
                    for op in itertools.chain(gxy("G1", self.coordinates[-2], travel),
                                              gz(self.z + self.retract),
                                              gxy(turn, travel, swivel,
                                                  center=self.coordinates[-2]),
                                              gz(self.z)):
                        yield op
            if self.skipping:
                self.coordinates.pop()
                if self.CRITICAL_PARAMETER.search(code):
                    # detect a shortcoming of the current implementation
                    logging.error("skipping feedrate or speed: %s", code)
                else:
                    logging.debug("skipping: %s", code)
            else:
                yield self.BLOCK
            while len(self.coordinates) > 3:
                self.coordinates.pop(0)
        else:
            logging.error("unexpected motion code %i", speed)
            yield self.BLOCK

def never_raise_blade(op):
    return not isinstance(op, basestring) or not op.startswith("G0Z")

def length_tuple(value, unit):
    if unit == 1:
        return (value / 25.4, value)
    else:
        return (value, value * 25.4)

def cli():
    # 1" = 25.4mm
    MAX_THICKNESS = (
        (0.0625, 1.6),
        (0.25, 6.3)
    )
    MIN_RADIUS = (
        (0.0625, 1.6),
        (0.125, 3.1)
    )
    SAFE_RETRACT = (0.2, 5)
    KNIFE_ID = { "D1": 0, "D2": 1, "D3": 0, "D4": 1 }
    UNIT = { "mm": 1, "in": 0 }
    LOG_LEVEL = {
        "critical": logging.CRITICAL,
        "error": logging.ERROR,
        "warning": logging.WARNING,
        "info": logging.INFO,
        "debug": logging.DEBUG
    }
    args_parser = argparse.ArgumentParser()
    args_parser.add_argument("input", help = "input file")
    args_parser.add_argument("-o", "--output", help = "output file")
    args_parser.add_argument("-t", "--thickness", required=True,
                             help="material thickness, ex.: 5mm or 0.2in")
    args_parser.add_argument("-k", "--knife", choices=["D1", "D2", "D3", "D4"],
                             help="drag knife model", default="D4")
    args_parser.add_argument("-a", "--angle", type=int, default = 20,
                             help="rotate the blade if turn is more than angle (degrees)")
    args_parser.add_argument("-nr", action="store_true",
                             help="never raise the blade (helps debug paths)")
    args_parser.add_argument("-l", dest="log", help="log level", default="warning",
                             choices=["critical", "error", "warning", "info", "debug"])
    args = args_parser.parse_args()
    logging.basicConfig(level=LOG_LEVEL[args.log])
    if args.output is None:
        path, basename = os.path.split(args.input)
        parts = basename.split(".")
        parts[-2 if len(parts) > 1 else -1] += "_knife"
        foutput = os.path.join(path, ".".join(parts))
    else:
        foutput = args.output
    logging.info("%s -> %s", args.input, foutput)
    knife = KNIFE_ID[args.knife]
    t_match = re.match(r"^([0-9\.]+)(mm|in)?$", args.thickness)
    if t_match == None:
        logging.critical("invalid thickness, needs value and unit such as 5mm or 0.2in")
        sys.exit(2)
    thickness = float(t_match.group(1))
    if t_match.group(2):
        unit = UNIT[t_match.group(2)]
    else:
        unit = UNIT["mm"]
    if thickness > MAX_THICKNESS[knife][unit]:
        logging.warning("%f is too thick for knife %s", thickness, args.knife)
    if thickness > MIN_RADIUS[knife][unit]:
        radius = length_tuple(thickness, unit)
    else:
        radius = MIN_RADIUS[knife]
    if not 10 <= args.angle <= 90:
        logging.critical("angle must be between 10 and 90 degrees")
        sys.exit(2)
    if args.nr:
        logging.warning("will not raise the blade (for debugging paths)")
        select = never_raise_blade
    else:
        select = None
    return (args.input, foutput,
            radius, length_tuple(thickness * 0.9, unit),
            length_tuple(thickness + SAFE_RETRACT[unit], unit), args.angle, select)

def run(finput, foutput, radius, retract, safe_retract, angle, select):
    MOVE = re.compile(r"G0?([0123])")
    UNIT = re.compile(r"G2([01])")
    postprocess = motion(radius[1], retract[1], safe_retract[1], angle, select)
    for block in nc:
        raw, code = preprocess(block)
        adapted = (raw, "\n")
        unit = UNIT.search(code)
        if unit:
            u = int(unit.group(1))
            postprocess = motion(radius[u], retract[u], safe_retract[u], angle, select)
        move = MOVE.search(code)
        if move:
            adapted = postprocess(int(move.group(1)), code)
        for op in adapted:
            if op == postprocess.BLOCK:
                out.write(raw)
                out.write("\n")
            elif isinstance(op, basestring):
                out.write(op)

if __name__ == "__main__":
    finput, foutput, radius, retract, safe_retract, sharp_angle, select = cli()
    with open(finput, "r") as nc, open(foutput, "w") as out:
        run(nc, out, radius, retract, safe_retract, sharp_angle, select)