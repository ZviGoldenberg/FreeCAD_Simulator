import math, time
import platform
import Part, Sketcher, Draft
from FreeCAD import Vector, Matrix, Placement
from PySide import QtGui, QtCore
import os, json
import numpy as np
import pdb
import ctypes


def Link(doc, object, name, copies=0):
    link = doc.addObject('App::Link', name)
    link.setLink(object)
    if copies > 1: link.ElementCount = copies
    return link


def LinkGroup(doc, parts, name):
    group = doc.addObject('App::DocumentObjectGroup', name + '_')
    group.Group = parts
    group.ViewObject.Visibility = False
    link = Link(doc, group, name)
    return link


def Mark(doc, visibility=True):
    mark = Draft.makePolygon(3, radius=5, inscribed=True, face=True, support=None)
    mark.ViewObject.ShapeColor = (0., 0., 0.)
    mark.ViewObject.Visibility = visibility
    return mark


def CirclePoint(radius, angle):
    return Vector(radius * math.cos(angle), radius * math.sin(angle), 0)


def RotaryScale(doc, name, radius):
    sketch = doc.addObject('Sketcher::SketchObject', name)
    sketch.Geometry = [Part.LineSegment(CirclePoint(radius, 2 * math.pi * i / 72),
                                        CirclePoint(radius + (5 if i % 9 == 0 else 2.5), 2 * math.pi * i / 72)) for i in
                       range(72)]
    sketch.ViewObject.PointColor, sketch.ViewObject.LineColor = (0., 0., 0.), (0., 0., 0.)
    sketch.ViewObject.LineWidth, sketch.ViewObject.PointSize = 2, 2
    sketch.ViewObject.Visibility = True
    return sketch


def Workpiece(doc, dim):
    # see file:///C:\Projects\ACS\5axes\AcsStageModel\FreeCad\workpiece.pdf
    # dim[0] is a distance between the machine zero and the workpiece bottom plain
    body = doc.addObject('PartDesign::Body', 'Workpiece')
    xy = doc.getObject('XY_Plane')
    cyl1 = body.newObject('PartDesign::AdditiveCylinder', 'PCylinder1')
    cyl1.Radius, cyl1.Height = dim[3], dim[4]
    cyl1.Support, cyl1.MapMode = xy, 'ObjectXY'
    cyl1.AttachmentOffset.Base = Vector(0, 0, -dim[0])
    cyl2 = body.newObject('PartDesign::AdditiveCylinder', 'PCylinder2')
    cyl2.Radius, cyl2.Height = dim[5], dim[6]
    cyl2.Support, cyl2.MapMode = xy, 'ObjectXY'
    cyl2.AttachmentOffset.Base = Vector(0, 0, -dim[0] + dim[4])
    cyl3 = body.newObject('PartDesign::AdditiveCylinder', 'PCylinder3')
    cyl3.Radius, cyl3.Height = dim[7], dim[1] - dim[2] - dim[4] - dim[6]
    cyl3.Support, cyl3.MapMode = xy, 'ObjectXY'
    cyl3.AttachmentOffset.Base = Vector(0, 0, -dim[0] + dim[4] + dim[6])
    sph = body.newObject('PartDesign::AdditiveSphere', 'PSphere')
    sph.Radius, sph.Angle1 = dim[2], 0
    sph.Support, sph.MapMode = xy, 'ObjectXY'
    sph.AttachmentOffset.Base = Vector(0, 0, -dim[0] + dim[1] - dim[2])
    body.ViewObject.Visibility = False
    return body


def Indexer(doc, dim):
    # see file:///C:\Projects\ACS\5axes\AcsStageModel\FreeCad\acsstage.pdf
    body = doc.addObject('PartDesign::Body', 'Indexer')
    xy = doc.getObject('XY_Plane')
    cyl2 = body.newObject('PartDesign::AdditiveCylinder', 'ICylinder')
    cyl2.Radius, cyl2.Height = dim[0], dim[1]
    cyl2.Support, cyl2.MapMode = xy, 'ObjectXY'
    cyl2.AttachmentOffset.Base = Vector(0, 0, -dim[1] - dim[2])

    for i in range(6):
        cyl = body.newObject('PartDesign::SubtractiveCylinder', 'ICyl')
        cyl.Radius, cyl.Height = 5, dim[1]
        cyl.Support, cyl.MapMode = xy, 'ObjectXY'
        cyl.AttachmentOffset.Base = Vector(dim[0] / 2 * math.sin(i * math.pi / 3),
                                           dim[0] / 2 * math.cos(i * math.pi / 3), -dim[1] - dim[2])

    mark = Mark(doc)
    mark.Support, mark.MapMode = xy, 'ObjectXY'
    mark.AttachmentOffset = Placement(Vector(0, -dim[0] + 5, -dim[2] + 0.001), Vector(0, 0, 1), -90)
    body.addObject(mark)
    body.ViewObject.Visibility = False
    return body


def Gimbal(doc, dim):
    # see file:///C:\Projects\ACS\5axes\AcsStageModel\FreeCad\acsstage.pdf
    body = doc.addObject('PartDesign::Body', 'Gimbal')
    body.Placement.Rotation.Axis = Vector(0, 1, 0)
    body.ViewObject.Visibility = False

    xy = doc.getObject('XY_Plane')
    xz = doc.getObject('XZ_Plane')
    yz = doc.getObject('YZ_Plane')

    g001 = body.newObject('PartDesign::AdditiveBox', 'G001')
    g001.Length, g001.Width, g001.Height = dim[1], dim[0], dim[7]
    g001.Support, g001.MapMode = xy, 'ObjectXY'
    g001.AttachmentOffset.Base = Vector(-dim[1] / 2, -dim[3], -dim[9])

    # see file:///C:\Projects\ACS\5axes\AcsStageModel\FreeCad\gimbalgeometry.jpg
    a, b, r = dim[9] - dim[7], dim[1] / 2, dim[10]
    d_2 = a * a + b * b
    d, l = math.sqrt(d_2), math.sqrt(d_2 - r * r)
    h, s = l * (a * l + b * r) / d_2, b - l * (b * l - a * r) / d_2
    g002 = body.newObject('PartDesign::AdditiveWedge', 'G002')
    g002.Xmin, g002.Xmax, g002.Ymin, g002.Ymax, g002.Zmin, g002.Zmax = -dim[3], dim[0] - dim[3], 0, h, -dim[1] / 2, dim[
        1] / 2
    g002.X2min, g002.X2max, g002.Z2min, g002.Z2max = g002.Xmin, g002.Xmax, -s, s
    g002.Support, g002.MapMode = xy, 'ObjectYZ'
    g002.AttachmentOffset.Base = Vector(0, dim[7] - dim[9], 0)

    g003 = body.newObject('PartDesign::AdditiveCylinder', 'G003')
    g003.Radius, g003.Height = r, dim[0]
    g003.Support, g003.MapMode = xz, 'ObjectXY'
    g003.AttachmentOffset.Base = Vector(0, 0, -dim[3])

    if dim[12] > 0:
        g004 = body.newObject('PartDesign::AdditiveCylinder', 'G004')
        g004.Radius, g004.Height = dim[11], dim[0] + 2 * dim[12]
        g004.Support, g004.MapMode = xz, 'ObjectXY'
        g004.AttachmentOffset.Base = Vector(0, 0, -dim[3] - dim[12])
        g005 = body.newObject('PartDesign::SubtractiveCylinder', 'G005')
        g005.Radius, g005.Height = 5, dim[0] + 2 * dim[12]
        g005.Support, g005.MapMode = xz, 'ObjectXY'
        g005.AttachmentOffset.Base = Vector(0, 0, -dim[3] - dim[12])
        g006 = Mark(doc)
        g006.Support, g006.MapMode = xz, 'FlatFace'
        g006.AttachmentOffset = Placement(Vector(0, -dim[11] + 5, dim[3] + dim[12] + 0.001), Vector(0, 0, 1), -90)
        body.addObject(g006)
        g007 = doc.copyObject(g006, False)
        g007.AttachmentOffset.Base = Vector(0, -dim[11] + 5, -dim[0] + dim[3] - dim[12] - 0.001)
        body.addObject(g007)
    else:
        g004 = body.newObject('PartDesign::SubtractiveCylinder', 'G004')
        g004.Radius, g004.Height = dim[11], dim[0]
        g004.Support, g004.MapMode = xz, 'ObjectXY'
        g004.AttachmentOffset.Base = Vector(0, 0, -dim[3])
        g005 = RotaryScale(doc, 'G005', dim[11])
        g005.Support, g005.MapMode = xz, 'FlatFace'
        g005.AttachmentOffset = Placement(Vector(0, 0, dim[3] + 0.001), Vector(0, 0, 1), 0)
        body.addObject(g005)
        g006 = doc.copyObject(g005, False)
        g006.AttachmentOffset.Base = Vector(0, 0, -dim[0] + dim[3] - 0.001)
        body.addObject(g006)
        g007 = Mark(doc)
        g007.Support, g007.MapMode = xz, 'FlatFace'
        g007.AttachmentOffset = Placement(Vector(0, -dim[11] - 5, dim[3] + 0.001), Vector(0, 0, 1), 90)
        body.addObject(g007)
        g008 = doc.copyObject(g007, False)
        g008.AttachmentOffset.Base = Vector(0, -dim[11] - 5, -dim[0] + dim[3] - 0.001)
        body.addObject(g008)

    if dim[4] >= 0:
        pass
    else:
        g010 = body.newObject('PartDesign::SubtractiveCylinder', 'G010')
        g010.Radius, g010.Height = dim[5], dim[2]
        g010.Support, g010.MapMode = xy, 'ObjectXY'
        g010.AttachmentOffset.Base = Vector(0, 0, -dim[9])
        g011 = RotaryScale(doc, 'G011', dim[5])
        g011.Support, g011.Placement = xy, Placement(Vector(0, 0, dim[2] - dim[9]), Vector(0, 0, 1), 0)
        body.addObject(g011)
        g012 = Mark(doc)
        g012.Support, g012.Placement = xy, Placement(Vector(0, -dim[5] - 5, dim[2] - dim[9]), Vector(0, 0, 1), 90)
        body.addObject(g012)

    g020 = body.newObject('PartDesign::SubtractiveBox', 'G020')
    g020.Length, g020.Width, g020.Height = dim[1], dim[0] - 2 * dim[6], dim[9] + dim[10]
    g020.Support, g020.MapMode = xy, 'ObjectXY'
    g020.AttachmentOffset.Base = Vector(-g020.Length / 2, dim[6] - dim[3], dim[2] - dim[9])
    g020.Refine = True
    if dim[13] > 0:
        g021 = body.newObject('PartDesign::SubtractiveCylinder', 'G021')
        g021.Radius, g021.Height = dim[13], dim[0] - 2 * dim[6]
        g021.Support, g021.MapMode = xz, 'ObjectXY'
        g021.AttachmentOffset.Base = Vector(0, 0, -dim[3] - dim[12])

    return body


def YBase(doc, dim):
    # see file:///C:\Projects\ACS\5axes\AcsStageModel\FreeCad\acsstage.pdf
    body = doc.addObject('PartDesign::Body', 'YBase')
    box = body.newObject('PartDesign::AdditiveBox', 'YBox')
    box.Length, box.Width, box.Height = dim[1], dim[0], dim[2]
    box.Support, box.MapMode = doc.getObject('XY_Plane'), 'ObjectXY'
    box.AttachmentOffset.Base = Vector(-box.Length / 2, -box.Width / 2, -dim[3])
    body.ViewObject.Visibility = False
    return body


def Head(doc, dim):
    # see file:///C:\Projects\ACS\5axes\AcsStageModel\FreeCad\acsstage.pdf
    xy = doc.getObject('XY_Plane')
    body = doc.addObject('PartDesign::Body', 'Head')
    box = body.newObject('PartDesign::AdditiveBox', 'HBox')
    box.Length, box.Width, box.Height = dim[5], dim[6], dim[7]
    box.Support, box.MapMode = xy, 'ObjectXY'
    box.AttachmentOffset.Base = Vector(-dim[5] + dim[4], -dim[6] / 2, dim[1] + dim[3])
    cyl1 = body.newObject('PartDesign::AdditiveCylinder', 'HCylinder')
    cyl1.Radius, cyl1.Height = dim[2], dim[3]
    cyl1.Support, cyl1.MapMode = xy, 'ObjectXY'
    cyl1.AttachmentOffset.Base = Vector(0, 0, dim[1])
    cyl2 = body.newObject('PartDesign::AdditiveCylinder', 'HCylinder1')
    cyl2.Radius, cyl2.Height = dim[0], dim[1]
    cyl2.Support, cyl2.MapMode = xy, 'ObjectXY'
    cyl2.AttachmentOffset.Base = Vector(0, 0, 0)
    cyl2.Refine = True
    body.ViewObject.Visibility = False
    return body


def Bed(doc, dim):
    # see file:///C:\Projects\ACS\5axes\AcsStageModel\FreeCad\acsstage.pdf
    body = doc.addObject('PartDesign::Body', 'Bed')
    box1 = body.newObject('PartDesign::AdditiveBox', 'BBox1')
    box1.Length, box1.Width, box1.Height = dim[0], dim[1], dim[2]
    box1.Support, box1.MapMode = doc.getObject('XY_Plane'), 'ObjectXY'
    box1.AttachmentOffset.Base = Vector(dim[8] - dim[0], -dim[1] / 2, -dim[9] - dim[2])
    box2 = body.newObject('PartDesign::AdditiveBox', 'BBox2')
    box2.Length, box2.Width, box2.Height = dim[3], dim[4], dim[5]
    box2.Support, box2.MapMode = doc.getObject('XY_Plane'), 'ObjectXY'
    box2.AttachmentOffset.Base = Vector(dim[8] - dim[0], -dim[4] / 2, -dim[9])
    box3 = body.newObject('PartDesign::AdditiveBox', 'BBox3')
    box3.Length, box3.Width, box3.Height = dim[6], dim[4], dim[7]
    box3.Support, box3.MapMode = doc.getObject('XY_Plane'), 'ObjectXY'
    box3.AttachmentOffset.Base = Vector(dim[8] - dim[0], -dim[4] / 2, -dim[9] + dim[5] - dim[7])
    box3.Refine = True
    body.ViewObject.Visibility = False
    return body


def Build(doc, g, b):
    # create components
    # see file:///C:\Projects\ACS\5axes\AcsStageModel\FreeCad\workpiece.pdf
    # The first parameter is a distance between the machine zero and the workpiece bottom plain
    workpiece = Workpiece(doc, (55, 70.91708, 38, 20, 3, 40, 7.5, 22))
    # see file:///C:\Projects\ACS\5axes\AcsStageModel\FreeCad\acsstage.pdf
    indexer = Indexer(doc, (60, 25, g))
    gimbal = Gimbal(doc, (278, 130, 25, 139, -25, 60, 19, 40, 50, g + 25, 49, 32.5, 20, 0))
    gimbalframe = Gimbal(doc, (
    318, 130, 25, 159, 0, 0, 20, 65, 0, g + 25 + 25, 49, 32.5, -20, math.sqrt(77 * 77 + 130 * 130 / 4.)))
    gimbalframe.Label = 'GimbalFrame'
    ybase = YBase(doc, (550, 130, 40, g + 25 + 25 + 40))
    head = Head(doc, (50, 55, 39, 80, 39, 202, 158, 154))
    # bed = Bed(doc, (750,550,250,200,200,837,112,158,720,275,102))
    bed = Bed(doc, (750, 550, 250, 162, 158, 837, 312, 600, 275, g + 25 + 25 + 40))

    # combine into mechanisms
    trace = Draft.makeWire([Vector(0, 0, 0), Vector(0, 0, -g)], closed=False, face=False)
    trace.ViewObject.Visibility = False
    tracelink = Link(doc, trace, 'Trace_')
    workpiecegroup = LinkGroup(doc, (workpiece), 'WorkpieceGroup')
    indexergroup = LinkGroup(doc, (indexer, workpiecegroup, tracelink), 'IndexerGroup')
    gimbalgroup = LinkGroup(doc, (gimbal, indexergroup), 'GimbalGroup')
    gimbalgroup.Placement.Rotation.Axis = Vector(0, 1, 0)
    ygroup = LinkGroup(doc, (gimbalframe, gimbalgroup), 'YGroup')
    xgroup = LinkGroup(doc, (ybase, ygroup), 'XGroup')
    beam = Draft.makeWire([Vector(0, 0, 0), Vector(0, 0, -b)])
    beam.ViewObject.Visibility = False
    beamlink = Link(doc, beam, 'Beam_')
    zgroup = LinkGroup(doc, (head, beamlink), 'ZGroup')
    machine = LinkGroup(doc, (bed, xgroup, zgroup), 'Machine')

    # colorize
    indexer.ViewObject.ShapeColor = (1.00, 0.83, 0.14)
    gimbal.ViewObject.ShapeColor = (0.79, 1.00, 0.04)
    head.ViewObject.ShapeColor = (84. / 255, 193. / 255, 1.)
    ybase.ViewObject.ShapeColor = (1., 1., 127. / 255)
    gimbalframe.ViewObject.ShapeColor = (1., 170. / 255, 1.)
    trace.ViewObject.LineColor = (1., 0., 0.)
    trace.ViewObject.PointColor = (1., 0., 0.)
    beam.ViewObject.LineColor = (1., 1., 1.)
    return (machine, xgroup, ygroup, zgroup, gimbalgroup, indexergroup, beamlink, tracelink, workpiecegroup)


class Machine:
    def __init__(self, doc, axesinfo):
        FreeCAD.Console.PrintMessage('>>>>Machine Start\n')
        self.doc = doc
        self.beamlength = 127
        self.g = 52  # distance between the center of table and B rotation axis
        self.machine, self.xgroup, self.ygroup, self.zgroup, self.gimbalgroup, self.indexergroup, self.beam, self.trace, self.workpiece = Build(
            doc, self.g, self.beamlength)
        self.mg2l, self.ml2g, self.mvalid = Matrix(), Matrix(), True
        self.totrace, self.tracepoints = False, []
        self.dummytrace = [Vector(0, 0, -53), Vector(0, 0, -53.001)]
        self.trace.Points = self.dummytrace
        self.beamlength = -self.beam.LinkedObject.Points[1].z
        self.axesinfo = axesinfo
        FreeCAD.Console.PrintMessage('>>>>Machine End\n')

    @property
    def G2L(self):
        if self.mvalid: return self.mg2l
        self.ml2g = self.xgroup.Placement.Matrix * self.ygroup.Placement.Matrix * self.gimbalgroup.Placement.Matrix * self.indexergroup.Placement.Matrix
        self.mg2l = self.ml2g.inverse()
        self.mvalid = True
        return self.mg2l

    @property
    def BeamLength(self):
        return self.beamlength

    @BeamLength.setter
    def BeamLength(self, value):
        if (value is not None) and (not np.isnan(value)) and (value != self.beamlength):
            self.beamlength = value
            self.beam.LinkedObject.Points = [Vector(0, 0, 0), Vector(0, 0, -value)]
            self.doc.recompute()

    def BeamLengthSet(self, value):
        self.BeamLength = value

    @property
    def X(self):
        return -self.xgroup.Placement.Base.x

    @X.setter
    def X(self, value):
        if (value is not None) and (not np.isnan(value)) and (value != self.X):
            self.xgroup.Placement.Base.x = -value
            self.xgroup.recompute()
            self.mvalid = False

    @property
    def Y(self):
        return -self.ygroup.Placement.Base.y

    @Y.setter
    def Y(self, value):
        if (value is not None) and (not np.isnan(value)) and (value != self.Y):
            self.ygroup.Placement.Base.y = -value
            self.ygroup.recompute()
            self.mvalid = False

    @property
    def Z(self):
        return self.zgroup.Placement.Base.z

    @Z.setter
    def Z(self, value):
        if (value is not None) and (not np.isnan(value)) and (value != self.Z):
            self.zgroup.Placement.Base.z = value
            self.zgroup.recompute()
            self.mvalid = False

    @property
    def B(self):
        return -self.gimbalgroup.Placement.Rotation.Angle / math.pi * 180

    @B.setter
    def B(self, value):
        if (value is not None) and (not np.isnan(value)) and (value != self.B):
            self.gimbalgroup.Placement.Rotation.Angle = -value * math.pi / 180
            self.gimbalgroup.recompute()
            self.mvalid = False

    @property
    def C(self):
        return self.indexergroup.Placement.Rotation.Angle / math.pi * 180

    @C.setter
    def C(self, value):
        if (value is not None) and (not np.isnan(value)) and (value != self.C):
            self.indexergroup.Placement.Rotation.Angle = value * math.pi / 180
            self.indexergroup.recompute()
            self.mvalid = False

    @property
    def XYZBC(self):
        return (self.X, self.Y, self.Z, self.B, self.C, self.BeamLength)

    @XYZBC.setter
    def XYZBC(self, value):
        while len(value) < 6: value.append(None)
        self.X, self.Y, self.Z, self.B, self.C, self.BeamLength = value
        if self.totrace: self.Trace()

    def setTrace(self, value):
        self.totrace = value

    def resetTrace(self):
        self.tracepoints = []
        self.trace.Points = self.dummytrace
        self.doc.recompute()

    #		self.trace.ViewObject.Visibility = False
    def setWorkpiece(self, value):
        self.workpiece.ViewObject.LinkVisibility = value
        self.workpiece.ViewObject.Visibility = False

    def Trace(self):
        v = self.G2L.multiply(Vector(0, 0, self.Z - self.beamlength))
        self.tracepoints.append(v)
        if len(self.tracepoints) >= 2:
            self.trace.Points = self.tracepoints
            self.trace.Closed = False
            self.doc.recompute()

    def GetKinematicMatricesByPrincipal(self, b, c):
        b, c = b * math.pi / 180, c * math.pi / 180
        sb, cb, sc, cc = math.sin(b), math.cos(b), math.sin(c), math.cos(c)
        direct = np.array(
            [[sb * cc, sb * sc, -cb, 0, 0, 0], [-sc, cc, 0, 0, 0, 0], [cb * cc, cb * sc, sb, 0, 0, self.BeamLength],
             [0, 0, 0, -1, 0, 180.0 / 2], [0, 0, 0, 0, -1, math.copysign(180.0, c)], [0, 0, 0, 0, 0, 1]], dtype=float)
        feedback = np.array([[sb * cc, -sc, cb * cc, 0, 0, -self.BeamLength * cb * cc],
                             [sb * sc, cc, cb * sc, 0, 0, -self.BeamLength * cb * sc],
                             [-cb, 0, sb, 0, 0, -self.BeamLength * sb], [0, 0, 0, -1, 0, 180.0 / 2],
                             [0, 0, 0, 0, -1, math.copysign(180.0, c)], [0, 0, 0, 0, 0, 1]], dtype=float)
        return direct, feedback

    def GetKinematicMatricesByMachine(self, b, c):
        b, c = b * math.pi / 180, c * math.pi / 180  # Convert to radians
        sb, cb, sc, cc = math.sin(b), math.cos(b), math.sin(c), math.cos(c)
        direct = np.array([[cb * cc, -cb * sc, -sb, 0], [sc, cc, 0, 0], [sb * cc, -sb * sc, cb, self.BeamLength], [0, 0, 0, 1]], dtype=float)
        feedback = np.array(
            [[cb * cc, sc, sb * cc, -self.BeamLength * sb * cc], [-cb * sc, cc, -sb * sc, self.BeamLength * sb * sc],
             [-sb, 0, cb, -self.BeamLength * cb], [0, 0, 0, 1]], dtype=float)
        return direct, feedback


class MachineGui(QtGui.QMainWindow):
    def __init__(self, machine, lib):
        FreeCAD.Console.PrintMessage('>>>>Machine GUI Start\n')
        super(MachineGui, self).__init__()
        self.machine = machine
        self.lib = lib
        self.initUI()
        self.run = False
        self.runAxes = [True, True, True, True, True]
        self.XYZBC = self.machine.XYZBC
        self.directory = '.'

    def initUI(self):
        self.result = 'Cancelled'
        # define window		xLoc,yLoc,xDim,yDim
        self.setGeometry(10, 30, 250, 355)
        self.setWindowTitle("ACS Stage v.10")
        self.setWindowFlags(QtCore.Qt.WindowStaysOnTopHint)
        #		grid = QtGui.QGridLayout(self)
        self.beamlabel = QtGui.QLabel('Beam length', self)
        self.beamlabel.setGeometry(15, 5, 60, 20)
        self.beambox = QtGui.QLineEdit(f'{float(self.machine.BeamLength):8.3f}', self)
        self.beambox.setGeometry(75, 5, 70, 20)
        self.beambox.textEdited.connect(lambda s: self.machine.BeamLengthSet(float(self.beambox.text())))
        self.labels, self.sliders, self.boxes, self.checks = [], [], [], []
        for ax in self.machine.axesinfo:
            i = len(self.labels)
            self.labels.append(QtGui.QLabel(ax[0], self))
            self.labels[i].setGeometry(15, 30 + 25 * i, 30, 20)
            self.sliders.append(QtGui.QSlider(QtCore.Qt.Horizontal, self))
            self.sliders[i].setGeometry(35, 30 + 25 * i, 100, 20)
            self.sliders[i].setMinimum(ax[1])
            self.sliders[i].setMaximum(ax[2])
            self.sliders[i].tag = i
            self.sliders[i].sliderMoved.connect(self.sliderMove)
            self.boxes.append(QtGui.QLineEdit('', self))
            self.boxes[i].setGeometry(145, 30 + 25 * i, 70, 20)
            self.boxes[i].textEdited.connect(self.axisEdited)
            self.boxes[i].tag = i
            self.checks.append(QtGui.QCheckBox(ax[0], self))
            self.checks[i].setGeometry(100 + 25 * i, 195, 40, 15)
            self.checks[i].setChecked(True)
            self.checks[i].stateChanged.connect(self.selectAxis)
            self.checks[i].tag = i
        self.zeroall = QtGui.QPushButton("Zero", self)
        self.zeroall.setGeometry(15, 160, 70, 30)
        self.zeroall.clicked.connect(self.ZeroAll)
        self.animate = QtGui.QPushButton("Animate", self)
        self.animate.setGeometry(15, 195, 70, 30)
        self.animate.clicked.connect(self.Animate)
        self.ratel = QtGui.QLabel('Rate', self)
        self.ratel.setGeometry(100, 210, 40, 20)
        self.rates = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        self.rates.setGeometry(145, 210, 55, 20)
        self.rates.setMinimum(0)
        self.rates.setMaximum(100)
        self.rates.setValue(100)
        # self.rates.valueChanged.connect(self.rateMove)
        self.pstart = QtGui.QPushButton("Start", self)
        self.pstart.setGeometry(15, 230, 70, 30)
        self.pstart.clicked.connect(self.Execute)
        self.pbox = QtGui.QLineEdit('', self)
        self.pbox.setGeometry(90, 235, 95, 20)
        self.pbrowse = QtGui.QPushButton("Browse", self)
        self.pbrowse.setGeometry(190, 235, 50, 20)
        self.pbrowse.clicked.connect(self.FileSelect)
        self.trace = QtGui.QCheckBox('Trace', self)
        self.trace.setGeometry(15, 265, 50, 20)
        self.trace.setChecked(True)
        self.machine.setTrace(True)
        self.trace.stateChanged.connect(lambda s: self.machine.setTrace(self.trace.isChecked()))
        self.cleartrace = QtGui.QPushButton("Clear trace", self)
        self.cleartrace.setGeometry(65, 265, 70, 20)
        self.cleartrace.clicked.connect(lambda s: self.machine.resetTrace())
        self.workpiece = QtGui.QCheckBox('Workpiece', self)
        self.workpiece.setGeometry(15, 285, 80, 20)
        self.workpiece.setChecked(True)
        self.workpiece.stateChanged.connect(lambda s: self.machine.setWorkpiece(self.workpiece.isChecked()))
        self.usedll = QtGui.QCheckBox('Use DLL', self)
        self.usedll.setGeometry(15, 305, 80, 20)
        if self.lib is not "":
            self.usedll.setChecked(False)
        else:
            self.usedll.setEnabled(False)
        self.bclose = QtGui.QPushButton("Close", self)
        self.bclose.setGeometry(190, 325, 50, 20)
        self.bclose.clicked.connect(lambda s: self.close())
        self.ZeroAll()
        self.show()

    def closeEvent(self, event):
        App.closeDocument('AcsStage')
        mw = FreeCADGui.getMainWindow()
        mw.showMaximized()
        event.accept()

    def SetMachineAxis(self, i, v):
        pos = [None, None, None, None, None, None]
        pos[i] = v
        self.machine.XYZBC = pos

    def sliderMove(self):
        i = self.sender().tag
        v = self.sliders[i].value()
        self.boxes[i].setText(f'{float(v):8.3f}')
        self.SetMachineAxis(i, v)

    def axisEdited(self):
        i = self.sender().tag
        v = float(self.boxes[i].text())
        self.sliders[i].setValue(v)
        self.SetMachineAxis(i, v)

    @property
    def XYZBC(self):
        v = [s.value for s in self.sliders]
        v.append(float(self.beambox.getText()))
        return v

    @XYZBC.setter
    def XYZBC(self, value):
        for v, s, b in zip(value, self.sliders, self.boxes):
            s.setValue(v)
            b.setText(f'{float(v):8.3f}')
        if len(value) > 5 and (value[5] is not None) and (not np.isnan(value[5])):
            self.beambox.setText(f'{float(value[5]):8.3f}')

    @property
    def Run(self):
        return self.run

    @Run.setter
    def Run(self, value):
        self.run = value
        self.animate.setText('Stop' if self.run else 'Animate')
        self.animate.setStyleSheet('color: red;' if self.run else 'color: black;')
        self.pstart.setText('Stop' if self.run else 'Start')
        self.pstart.setStyleSheet('color: red;' if self.run else 'color: black;')

    def ZeroAll(self):
        inside_table_part = 3
        WP_Height = 70.9 - inside_table_part
        Z0 = (WP_Height - self.machine.g + self.machine.beamlength) if self.workpiece.isChecked() else 0
        self.XYZBC = (0, 0, Z0, 0, 0, None)
        self.machine.XYZBC = (0, 0, Z0, 0, 0, None)

    # def rateMove(self):
    # 	self.runIncr = 0.2+self.rates.value()*0.01*1.8
    def selectAxis(self):
        i = self.sender().tag
        self.runAxes[i] = self.checks[i].isChecked()

    def AnimationStep(self, value, incr, min_, max_):
        if value <= min_:
            incr = 1
        elif value >= max_:
            incr = -1
        runIncr = 0.2 + self.rates.value() * 0.01 * 1.8
        return (value + incr * runIncr, incr)

    def Animate(self):
        self.Run = not self.Run
        if not self.Run: return
        value = list(self.machine.XYZBC)
        incr = [1, 1, 1, 1, 1]
        while self.Run:
            for i, a in zip(range(5), self.machine.axesinfo):
                if (self.runAxes[i]):
                    value[i], incr[i] = self.AnimationStep(value[i], incr[i], a[1], a[2])
            self.XYZBC = value
            self.machine.XYZBC = value
            FreeCAD.Gui.updateGui()
            time.sleep(0.01)

    def FileSelect(self):
        filename, filter = QtGui.QFileDialog.getOpenFileName(parent=self, caption='Open file', dir=self.directory,
                                                             filter='CSV file (*.csv)')
        FreeCAD.Console.PrintMessage(filename + '\n')
        if filename:
            self.directory = os.path.dirname(filename)
            self.pbox.setText(os.path.basename(filename))

    def Spherical2projections(self, spheric):
        theta, phi = spheric * math.pi / 180
        st, ct, sp, cp = math.sin(theta), math.cos(theta), math.sin(phi), math.cos(phi)
        return [ct * cp, ct * sp, st]

    def ToPlusMinusPi(self, a):
        return a if (a <= 180) and (a >= -180) else a + 360 if a < 0 else a - 360

    def Projections2spherical(self, proj, expectedphi):
        print('proj:', proj)
        d = np.linalg.norm(proj)
        dx, dy, dz = proj / d
        if dz > 0.9999999:
            return 90.0, expectedphi
        else:
            theta, phi = math.asin(dz) * 180.0 / math.pi, math.atan2(dy, dx) * 180.0 / math.pi
            print('spher0:', theta, phi, expectedphi)
            if not np.isnan(expectedphi) and abs(self.ToPlusMinusPi(phi - expectedphi)) > 90:
                theta = 180 - theta
                phi = phi + 180 if phi <= 0 else phi - 180
                print('spher1:', theta, phi)
            return theta, phi

    def GetFixtureMatrices(self, offs):
        g = self.machine.g
        x, y, z, a, b, c = offs[0], offs[1], offs[2], offs[3] * math.pi / 180, offs[4] * math.pi / 180, offs[
            5] * math.pi / 180
        sa, ca, sb, cb, sc, cc = math.sin(a), math.cos(a), math.sin(b), math.cos(b), math.sin(c), math.cos(c)
        direct = np.array([[cb * cc, -cb * sc, sb, x], [ca * sc + sa * sb * cc, ca * cc - sa * sb * sc, -sa * cb, y],
                           [sa * sc - ca * sb * cc, sa * cc + ca * sb * sc, ca * cb, z - g], [0, 0, 0, 1]], dtype=float)
        feedback = np.array([[cb * cc, ca * sc + sa * sb * cc, sa * sc - ca * sb * cc],
                             [-cb * sc, ca * cc - sa * sb * sc, sa * cc + ca * sb * sc], [sb, -sa * cb, ca * cb]],
                            dtype=float)
        feedback = np.append(feedback, -feedback @ [[x], [y], [z - g]], 1)
        feedback = np.append(feedback, [[0.0, 0.0, 0.0, 1.0]], 0)
        return direct, feedback

    def Execute(self):
        # !!!! Only supported coordinates are 'machine' and 'mixed' !!!!!
        self.Run = not self.Run
        if not self.Run: return
        with open(self.directory + '\\' + self.pbox.text(), 'r', newline='') as f:
            row = f.readline()
            coor = 'machine'
            fixture, kinematic, ffixture, fkinematic = np.identity(4), np.identity(6), np.identity(4), np.identity(6)
            desc = None
            offs = [0, 0, 0, 0, 0, 0]
            principal1, principal2, principal3 = None, None, None
            if row.startswith('{'):
                desc = json.loads(row)
                coor = desc['coordinates']
                row = f.readline()
            if coor != 'machine':
                flen = desc['focallength']
            else:
                flen = self.machine.BeamLength
            if (coor == 'part') or (coor == 'mixed'):
                print(">>>>Part coordinates")
                offs = desc['offsets']
                print('offsets:', offs)
                fixture, ffixture = self.GetFixtureMatrices(offs)
                print('fixture:', fixture)
                print('ffixture:', ffixture)
                print('verify:', fixture @ ffixture)
                print('Beam length: ', flen)
                self.machine.BeamLength = flen
                if self.usedll.isChecked():
                    self.lib.SetFixtureOffsets(offs[0], offs[1], offs[2], offs[3], offs[4], offs[5])
                    self.lib.SetToolLength(flen)
            while row:
                if not self.Run: return
                els = row.replace(',', '\t').split('\t')
                if (len(els) < 5): break
                elsnp = np.array(els, dtype=float)
                if (len(elsnp) > 5):
                    flen = elsnp[5]  # Receives length from the GUI window\
                vpart = np.append(elsnp[:5], 1.0)  # Switches the input vector to homogenous form -> vpart
                print('vpart:', vpart)
                vprincipal, vmachine = vpart, vpart  # Initialization of vprincipal, vmachine homogenous vectors
                if coor == 'part':
                    dx, dy, dz = self.Spherical2projections(vpart[3:5])
                    # print('dx/dy/dz:', dx, dy, dz)
                    t = fixture @ np.array([[vpart[0], dx], [vpart[1], dy], [vpart[2], dz], [1.0, 0.0]])
                    theta, phi = self.Projections2spherical(t[:3, 1], np.NaN if principal1 is None else principal1[4])
                    vprincipal = np.append(t[:3, 0], [theta, phi, 1.0])
                    principal3, principal2, principal1 = principal2, principal1, vprincipal
                    kinematic, fkinematic = self.machine.GetKinematicMatricesByPrincipal(vprincipal[3], vprincipal[4])
                    vmachine = kinematic @ vprincipal
                    fprincipal = fkinematic @ vmachine
                    dx, dy, dz = self.Spherical2projections(np.array([fprincipal[3], fprincipal[4]]))
                    t = ffixture @ np.array([[fprincipal[0], dx], [fprincipal[1], dy], [fprincipal[2], dz], [1.0, 0.0]])
                    theta, phi = self.Projections2spherical(t[:3, 1], vpart[4])
                    fpart = np.append(t[:3, 0], [theta, phi, 1.0])
                elif coor == 'principal':
                    kinematic, fkinematic = self.machine.GetKinematicMatricesByPrincipal(vprincipal[3], vprincipal[4])
                    vmachine = kinematic @ vprincipal
                    fprincipal = fkinematic @ vmachine
                    fpart = fprincipal
                elif coor == 'mixed':
                    if self.usedll.isChecked():
                        # calculate in dll
                        mixed_dll = np.array(vpart[:5], np.float64)
                        machine_dll = np.array([0, 0, 0, 0, 0], np.float64)
                        res = self.lib.DoDirectTransform(5, mixed_dll, 5, machine_dll)

                        #Testing
                        print('dll_Z:', machine_dll[2])
                        kinematics = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,], np.float64)
                        res = self.lib.GetKinematicMatrix(16, kinematics)
                        print('kinematics', kinematics)
                        fixture = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, ], np.float64)
                        res = self.lib.GetFixtureMatrix(16, fixture)
                        print('fixture', fixture)


                        feedback_dll = np.array([0, 0, 0, 0, 0], np.float64)
                        res = self.lib.DoFeedbackTransform(5, machine_dll, 5, feedback_dll)
                        vmachine = np.append(machine_dll, 1.0)
                        fpart = np.append(feedback_dll, 1.0)
                    else:
                        # calculate in python
                        print('vpart:', vpart)
                        vprincipal = fixture @ np.array(
                            [vpart[0], vpart[1], vpart[2], 1.0])  # Multiplication fixture by xyz1 input vector
                        print('principal:', vprincipal)
                        b, c = vpart[3] - offs[4], vpart[4] - offs[5]
                        kinematic, fkinematic = self.machine.GetKinematicMatricesByMachine(b, c)
                        print('kinematic:', kinematic)
                        vmachine = kinematic @ vprincipal
                        #print('vmachine:', vmachine)
                        print('internal_Z:', vmachine[2])
                        # Feedback transformations
                        fprincipal = fkinematic @ vmachine
                        fpart = ffixture @ fprincipal
                        vmachine = np.append(vmachine[:3], [b, c, 1.0])
                        # print('vmachine:', vmachine)
                        fprincipal = np.append(fprincipal[:3], [vpart[3], vpart[4], 1.0])
                        fpart = np.append(fpart[:3], [vpart[3], vpart[4], 1.0])
                elif coor == 'machine':
                    vmachine = vpart
                    vmachine[0] = -vmachine[0]
                    vmachine[1] = -vmachine[1]
                    vmachine[3] = -vmachine[3]
                    fpart = vpart
                elif coor == 'machine_dll':
                    vmachine = vpart
                    fpart = vpart
                dvec = vpart - fpart
                # print('vpart:', vpart)
                # print('fpart:', fpart)
                # print('difference:', dvec)
                if any(d > 1.e-6 for d in dvec): print('@@@@@@@@@@@@@@@@@@@@@@@@@@')
                vec = np.append(vmachine[:5], flen)
                # print('flen:', flen)
                self.XYZBC = vec
                self.machine.XYZBC = vec
                FreeCAD.Gui.updateGui()
                # time.sleep(1 / max(1., self.rates.value()))
                row = f.readline()
        self.Run = False


def InitDll():
    if platform.architecture()[0] == "32bit":
        lib = ctypes.CDLL("C:\SB4\import\StageModel_1.dll")
    if platform.architecture()[0] == "64bit":
        lib = ctypes.CDLL("C:\SB4\import\StageModel_1_x64.dll")
    lib.SetFixtureOffsets.restype = ctypes.c_int
    lib.SetFixtureOffsets.argtypes = [ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double,
                                      ctypes.c_double, ctypes.c_double]
    lib.DoDirectTransform.restype = ctypes.c_int
    lib.DoDirectTransform.argtypes = [ctypes.c_int, np.ctypeslib.ndpointer(dtype=np.float64), ctypes.c_int,
                                      np.ctypeslib.ndpointer(dtype=np.float64)]
    lib.DoFeedbackTransform.restype = ctypes.c_int
    lib.DoFeedbackTransform.argtypes = [ctypes.c_int, np.ctypeslib.ndpointer(dtype=np.float64), ctypes.c_int,
                                        np.ctypeslib.ndpointer(dtype=np.float64)]
    lib.SetToolLength.restype = ctypes.c_int
    lib.SetToolLength.argtypes = [ctypes.c_double]
    lib.SetToolRadius.restype = ctypes.c_int
    lib.SetToolRadius.argtypes = [ctypes.c_double]
    lib.IsFeedbackKinematicsSupported.restype = ctypes.c_int
    lib.IsFeedbackKinematicsSupported.argtypes = [np.ctypeslib.ndpointer(dtype=np.int32)]
    r = np.array([0], np.int32)
    lib.GetStageModelInd.restype = ctypes.c_int
    lib.GetStageModelInd.argtypes = [np.ctypeslib.ndpointer(dtype=np.int32)]
    lib.SetCoordinatesFormat.restype = ctypes.c_int
    lib.SetCoordinatesFormat.argtypes = [ctypes.c_int]
    lib.GetRotAxesModel.restype = ctypes.c_int
    lib.GetRotAxesModel.argtypes = [np.ctypeslib.ndpointer(dtype=np.int32)]

    #Testing
    lib.GetKinematicMatrix.restype = ctypes.c_int
    lib.GetKinematicMatrix.argtypes = [ctypes.c_int, np.ctypeslib.ndpointer(dtype=np.float64)]
    lib.GetFixtureMatrix.restype = ctypes.c_int
    lib.GetFixtureMatrix.argtypes = [ctypes.c_int, np.ctypeslib.ndpointer(dtype=np.float64)]
    return lib


doc = App.newDocument('AcsStage')
axesinfo = (('X', -200, 200), ('Y', -200, 200), ('Z', 0, 350), ('B', -90, 90), ('C', -180, 180))  # motion limits
machine = Machine(doc, axesinfo)
doc.recompute()
Gui.SendMsgToActiveView("ViewFit")
Gui.activeDocument().activeView().viewIsometric()
FreeCAD.Gui.updateGui()
mw = FreeCADGui.getMainWindow()
ev = QtGui.QKeyEvent(QtCore.QEvent.KeyPress, QtCore.Qt.Key_F11, QtCore.Qt.NoModifier)
QtGui.QApplication.sendEvent(mw, ev)
mw.showMinimized()
try:
    lib = InitDll()
except Exception as ex:
    lib = ""
    print('dll is not available')
gui = MachineGui(machine, lib)
