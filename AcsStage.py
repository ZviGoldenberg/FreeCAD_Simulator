import math, time
import Part, Sketcher, Draft
from FreeCAD import Vector, Matrix, Placement
from PySide import QtGui, QtCore
import os, json
import numpy as np
import pdb

def Link(doc, object, name, copies=0):
	link = doc.addObject('App::Link',name)
	link.setLink(object)
	if copies>1: link.ElementCount = copies
	return link

def Mark(doc, visibility=True):
	mark = Draft.makePolygon(3, radius=5, inscribed=True, face=True, support=None)
	mark.ViewObject.ShapeColor = (0.,0.,0.)
	mark.ViewObject.Visibility = visibility
	return mark

def RotaryScale(doc, name, radius):
	FreeCAD.Console.PrintMessage('>>>>Rotary Scale Start ('+name+')\n')
	body = doc.addObject('PartDesign::Body',name)
	mark = Mark(doc)
	mark.Placement = Placement(Vector(radius+5,0,0), Vector(0,0,1), 180)
	main = Draft.makeWire([Vector(radius,0,0), Vector(radius+5,0,0)])	
	mains = Draft.make_polar_array(main, number=8, angle=360.0, use_link=True)
	tick = Draft.makeWire([Vector(radius,0,0), Vector(radius+2.5,0,0)])	
	ticks = Draft.make_polar_array(tick, number=72, angle=360.0, use_link=True)
	body.Group = (mark, mains, ticks)
	body.ViewObject.Visibility = False
	FreeCAD.Console.PrintMessage('>>>>Rotary Scale End\n')
	return body

def Indexer(doc):
	body = doc.addObject('PartDesign::Body','Indexer')
	body.Placement.Rotation.Axis = Vector(0,0,1)

	lcs = body.newObject('PartDesign::CoordinateSystem', 'I_LCS')
	lcs.Placement = Placement(Vector(0,0,-52), Vector(0,0,1), 180)	

	cyl2 = body.newObject('PartDesign::AdditiveCylinder','ICylinder')
	cyl2.Radius,cyl2.Height = 60,25
	cyl2.Support,cyl2.MapMode = lcs,'ObjectXY'
	cyl2.AttachmentOffset.Base = Vector(0,0,-25)

	for i in range(6):
		cyl = body.newObject('PartDesign::SubtractiveCylinder','ICyl')
		cyl.Radius,cyl.Height = 5,25
		cyl.Support,cyl.MapMode = lcs,'ObjectXY'
		cyl.AttachmentOffset.Base = Vector(30*math.sin(i*math.pi/3),30*math.cos(i*math.pi/3),-25)

	mark = Mark(doc)
	mark.Support,mark.MapMode = lcs,'ObjectXY'
	mark.AttachmentOffset = Placement(Vector(0,55,0.1), Vector(0,0,1), 90)
	body.addObject(mark)
	body.ViewObject.Visibility = False
	return body

def Gimbal(doc):
	sketch = doc.addObject("Sketcher::SketchObject","GSketch")
	pc = ((-49,0),(-65,-52),(-65,-77),(65,-77),(65,-52),(49,0))
	points = [Vector(p[0],p[1],0) for p in pc]
	geom = [Part.LineSegment(points[i],points[i+1]) for i in range(len(points)-1)]
	geom.append(Part.ArcOfCircle(Part.Circle(Vector(0,0,0),Vector(0,0,1),49),0,math.pi))
	sketch.Geometry = geom
	n = len(geom)
	sketch.addConstraint(Sketcher.Constraint('Vertical',1)) 
	sketch.addConstraint(Sketcher.Constraint('Symmetric',1,1,3,2,-2)) 
	sketch.addConstraint(Sketcher.Constraint('Symmetric',1,2,3,1,-2)) 
	sketch.addConstraint(Sketcher.Constraint('Radius',n-1,49)) 
	sketch.addConstraint(Sketcher.Constraint('DistanceX',2,1,2,2,130)) 
	sketch.addConstraint(Sketcher.Constraint('DistanceY',2,1,-1,1,77)) 
	sketch.addConstraint(Sketcher.Constraint('DistanceY',1,1,1,2,-25)) 
	for i in range(n-2): sketch.addConstraint(Sketcher.Constraint('Coincident',i,2,i+1,1))
	sketch.addConstraint(Sketcher.Constraint('Coincident',n-1,3,-1,1))
	sketch.addConstraint(Sketcher.Constraint('Tangent',n-2,2,n-1,1))		
	sketch.addConstraint(Sketcher.Constraint('Tangent',n-1,2,0,1))

	body = doc.addObject('PartDesign::Body','Gimbal')
	body.Placement.Rotation.Axis = Vector(0,1,0)
	body.ViewObject.Visibility = False

	lcs = body.newObject('PartDesign::CoordinateSystem', 'G_LCS')
	lcs.Placement = Placement(Vector(0,0,-52), Vector(0,0,1), 180)	

	xz = doc.getObject('XZ_Plane')
	sketch.Support,sketch.MapMode = xz,'FlatFace'
	pad = body.newObject('PartDesign::Pad','GPad')
	pad.Profile,pad.Length,pad.Midplane = sketch,278,1

	cyl = body.newObject('PartDesign::AdditiveCylinder','GCylinder')
	cyl.Radius,cyl.Height = 32.5,314
	cyl.Support,cyl.MapMode = xz,'ObjectXY'
	cyl.AttachmentOffset.Base = Vector(0,0,-cyl.Height/2)

	cyl1 = body.newObject('PartDesign::SubtractiveCylinder','GCylinder1')
	cyl1.Radius,cyl1.Height = 6.5,314
	cyl1.Support,cyl1.MapMode = xz,'ObjectXY'
	cyl1.AttachmentOffset.Base = Vector(0,0,-cyl1.Height/2)

	box = body.newObject('PartDesign::SubtractiveBox','GBox')
	box.Length,box.Width,box.Height=130,250,150
	box.Support,box.MapMode = lcs,'ObjectXY'
	box.AttachmentOffset.Base = Vector(-box.Length/2,-box.Width/2,0)

	cyl2 = body.newObject('PartDesign::SubtractiveCylinder','GCylinder2')
	cyl2.Radius,cyl2.Height = 60,25
	cyl2.Support,cyl2.MapMode = lcs,'ObjectXY'
	cyl2.AttachmentOffset.Base = Vector(0,0,-25)

	sketch.ViewObject.Visibility = False
	return body

def GimbalFrame(doc):
	FreeCAD.Console.PrintMessage('>>>>Gimbal Frame Start\n')
	sketch = doc.addObject("Sketcher::SketchObject","FSketch")
	pc = ((-49,0),(-74,-52),(-74,-102.5),(74,-102.5),(74,-52),(49,0))
	pts = [Vector(p[0],p[1],0) for p in pc]
	sketch.Geometry = [Part.LineSegment(pts[i],pts[i+1]) for i in range(len(pts)-1)]
	sketch.addGeometry(Part.ArcOfCircle(Part.Circle(Vector(0,0,0),Vector(0,0,1),49),0,math.pi))
	n = len(pts)
	sketch.addConstraint(Sketcher.Constraint('Vertical',1)) 
	sketch.addConstraint(Sketcher.Constraint('Symmetric',1,1,3,2,-2)) 
	sketch.addConstraint(Sketcher.Constraint('Symmetric',1,2,3,1,-2)) 
	sketch.addConstraint(Sketcher.Constraint('Radius',n-1,49)) 
	sketch.addConstraint(Sketcher.Constraint('DistanceX',2,1,2,2,148)) 
	sketch.addConstraint(Sketcher.Constraint('DistanceY',2,1,-1,1,102.5)) 
	sketch.addConstraint(Sketcher.Constraint('DistanceY',1,1,1,2,-25)) 
	for i in range(n-2): sketch.addConstraint(Sketcher.Constraint('Coincident',i,2,i+1,1))
	sketch.addConstraint(Sketcher.Constraint('Coincident',n-1,3,-1,1))
	sketch.addConstraint(Sketcher.Constraint('Tangent',n-2,2,n-1,1))		
	sketch.addConstraint(Sketcher.Constraint('Tangent',n-1,2,0,1))

	body = doc.addObject('PartDesign::Body','GimbalFrame')
	xz = doc.getObject('XZ_Plane')
	sketch.Support,sketch.MapMode = xz,'FlatFace'
	pad = body.newObject('PartDesign::Pad','FPad')
	pad.Profile,pad.Length,pad.Midplane= sketch,314,1

	cyl = body.newObject('PartDesign::SubtractiveCylinder','FCylinder')
	cyl.Radius,cyl.Height = 32.5,314
	cyl.Support,cyl.MapMode = xz,'ObjectXY'
	cyl.AttachmentOffset.Base = Vector(0,0,-cyl.Height/2)

	fcyl1 = body.newObject('PartDesign::SubtractiveCylinder','FCylinder1')
	fcyl1.Radius,fcyl1.Height = 101,280
	fcyl1.Support,fcyl1.MapMode = xz,'ObjectXY'
	fcyl1.AttachmentOffset.Base = Vector(0,0,-fcyl1.Height/2)

	box = body.newObject('PartDesign::SubtractiveBox','FBox')
	box.Length,box.Width,box.Height=148,154,280
	box.Support,box.MapMode = xz,'ObjectXY'
	box.AttachmentOffset.Base = Vector(-box.Length/2,-box.Width/2,-box.Height/2)

	sketch.ViewObject.Visibility = False
	body.ViewObject.Visibility = False
	FreeCAD.Console.PrintMessage('>>>>Gimbal Frame End\n')
	return body

def YBase(doc):
	body = doc.addObject('PartDesign::Body','YBase')
	box = body.newObject('PartDesign::AdditiveBox','YBox')
	box.Length,box.Width,box.Height=158,40,550
	box.Support,box.MapMode = doc.getObject('XZ_Plane'),'ObjectXY'
	box.AttachmentOffset.Base = Vector(-77,-142,-box.Height/2)
	body.ViewObject.Visibility = False
	return body
	
def XBase(doc):
	body = doc.addObject('PartDesign::Body','XBase')
	box = body.newObject('PartDesign::AdditiveBox','XBox')
	box.Length,box.Width,box.Height=450,40,550
	box.Support,box.MapMode = doc.getObject('YZ_Plane'),'ObjectXY'
	box.AttachmentOffset.Base = Vector(-box.Length/2,-182,-box.Height/2)
	body.ViewObject.Visibility = False
	return body

def ZBase(doc):
	body = doc.addObject('PartDesign::Body','ZBase')
	box = body.newObject('PartDesign::AdditiveBox','ZBox')
	box.Length,box.Width,box.Height=275-198+35,158,520
	box.Support,box.MapMode = doc.getObject('XY_Plane'),'ObjectXY'
	box.AttachmentOffset.Base = Vector(-275,-box.Width/2,135)
	body.ViewObject.Visibility = False
	return body

def Head(doc):
	xy = doc.getObject('XY_Plane')
	body = doc.addObject('PartDesign::Body','ZBase')
	box = body.newObject('PartDesign::AdditiveBox','ZBox')
	box.Length,box.Width,box.Height=39+198-35,158,154
	box.Support,box.MapMode = xy,'ObjectXY'
	box.AttachmentOffset.Base = Vector(-198+35,-box.Width/2,135)
	body.ViewObject.Visibility = False
	cyl1 = body.newObject('PartDesign::AdditiveCylinder','GCylinder')
	cyl1.Radius,cyl1.Height = 39,80
	cyl1.Support,cyl1.MapMode = xy,'ObjectXY'
	cyl1.AttachmentOffset.Base = Vector(0,0,55)
	cyl2 = body.newObject('PartDesign::AdditiveCylinder','GCylinder')
	cyl2.Radius,cyl2.Height = 50,55
	cyl2.Support,cyl2.MapMode = xy,'ObjectXY'
	cyl2.AttachmentOffset.Base = Vector(0,0,0)
	return body

def Bed(doc):
	body = doc.addObject('PartDesign::Body','Bed')
	box1 = body.newObject('PartDesign::AdditiveBox','BBox1')
	box1.Length,box1.Width,box1.Height=750,550,250
	box1.Support,box1.MapMode = doc.getObject('XY_Plane'),'ObjectXY'
	box1.AttachmentOffset.Base = Vector(275-750,-275,-182-250)
	body.ViewObject.Visibility = False
	box2 = body.newObject('PartDesign::AdditiveBox','BBox2')
	box2.Length,box2.Width,box2.Height=200,200,182+135+520
	box2.Support,box2.MapMode = doc.getObject('XY_Plane'),'ObjectXY'
	box2.AttachmentOffset.Base = Vector(275-750,-box2.Width/2,-182)
	body.ViewObject.Visibility = False
	return body

def IndexerGroup(doc):
	FreeCAD.Console.PrintMessage('>>>>Indexer Group Start\n')
	indexer = Indexer(doc)
	trace = Draft.makeWire([],closed=False,face=False)	
	trace.ViewObject.Visibility = False
	tracelink = Link(doc,trace,'Trace_')
	group = doc.addObject('App::DocumentObjectGroup','IndexerGroup')
	group.Group = (indexer,tracelink)
	group.ViewObject.Visibility = False
	link = Link(doc,group,'IndexerGroup_')
	FreeCAD.Console.PrintMessage('>>>>Indexer Group End\n')
	return (link,indexer,tracelink)

def GimbalGroup(doc):
	FreeCAD.Console.PrintMessage('>>>>Gimbal Group Start\n')
	gimbal = Link(doc,Gimbal(doc),'Gimbal_')
	indexergroup,indexer,trace = IndexerGroup(doc)
	scale = Link(doc, RotaryScale(doc,'IndexerScale',60), 'IndexerScale_')
	scale.Placement = Placement(Vector(0,0,-51.9), Vector(0,0,1), -90)
	mark = Link(doc, Mark(doc, False), 'GimbalMark_', 2)
	mark.Placement = Placement(Vector(27.5,0,0),Vector(1,0,0),90)
	mark.ElementList[0].Placement.Base = Vector(0,0,157.1)
	mark.ElementList[1].Placement.Base = Vector(0,0,-157.1)
	group = doc.addObject('App::DocumentObjectGroup','GimbalGroup')
	group.Group = (gimbal,indexergroup,scale,mark)
	group.ViewObject.Visibility = False
	link = Link(doc,group,'GimbalGroup_')
	link.Placement.Rotation.Axis = Vector(0,1,0)
	FreeCAD.Console.PrintMessage('>>>>Gimbal Group End\n')
	return (link,gimbal,indexergroup,indexer,trace)

def YGroup(doc):
	FreeCAD.Console.PrintMessage('>>>>Y Group Start\n')
	frame = Link(doc, GimbalFrame(doc), 'GimbalFrame_')
	scale = Link(doc, RotaryScale(doc, 'GimbalScale', 32.5), 'GimbalScale_', 2)
	scale.Placement = Placement(Vector(0,0,0),Vector(1,0,0),90)
	scale.ElementList[0].Placement.Base = Vector(0,0,157.1)
	scale.ElementList[1].Placement.Base = Vector(0,0,-157.1)

	gimbalgroup,gimbal,indexergroup,indexer,trace = GimbalGroup(doc)
	group = doc.addObject('App::DocumentObjectGroup','YGroup')
	link = Link(doc,group,'YGroup_')
	group.Group = (frame,gimbalgroup,scale)
	group.ViewObject.Visibility = False
	FreeCAD.Console.PrintMessage('>>>>Y Group End\n')
	return (link,gimbalgroup,frame,gimbal,indexergroup,indexer,trace)

def XGroup(doc):
	FreeCAD.Console.PrintMessage('>>>>X Group Start\n')
	ybase = Link(doc, YBase(doc), 'YBase_')
	ygroup,gimbalgroup,frame,gimbal,indexergroup,indexer,trace = YGroup(doc)
	group = doc.addObject('App::DocumentObjectGroup','XGroup')
	link = Link(doc,group,'XGroup_')
	group.Group = (ybase,ygroup)
	group.ViewObject.Visibility = False
	FreeCAD.Console.PrintMessage('>>>>X Group End\n')
	return (link,ygroup,gimbalgroup,ybase,frame,gimbal,indexergroup,indexer,trace)

def ZGroup(doc):
	FreeCAD.Console.PrintMessage('>>>>Z Group Start\n')
	head = Link(doc, Head(doc), 'Head_')
	beam = Draft.makeWire([Vector(0,0,0), Vector(0,0,-52)])	
	beam.ViewObject.Visibility = False
	beamlink = Link(doc,beam,'Beam_')
	group = doc.addObject('App::DocumentObjectGroup','ZGroup')
	link = Link(doc,group,'ZGroup_')
	group.Group = (head,beamlink)
	group.ViewObject.Visibility = False
	FreeCAD.Console.PrintMessage('>>>>Z Group End\n')
	return (link,head,beamlink)

def BedGroup(doc):
	FreeCAD.Console.PrintMessage('>>>>Bed Group Start\n')
	xbase = Link(doc, XBase(doc), 'XBase_')
	zbase = Link(doc, ZBase(doc), 'ZBase_')
	bed = Link(doc, Bed(doc), 'Bed_')
	group = doc.addObject('App::DocumentObjectGroup','BaseGroup')
	link = Link(doc,group,'BaseGroup_')
	group.Group = (bed,xbase,zbase)
	group.ViewObject.Visibility = False
	FreeCAD.Console.PrintMessage('>>>>Bed Group End\n')
	return (link,bed,xbase,zbase)

def MachineGroup(doc):
	FreeCAD.Console.PrintMessage('>>>>Machine Group Start\n')
	bedgroup,bed,xbase,zbase = BedGroup(doc)
	xgroup,ygroup,gimbalgroup,ybase,frame,gimbal,indexergroup,indexer,trace = XGroup(doc)
	zgroup,head,beam = ZGroup(doc)
	group = doc.addObject('App::DocumentObjectGroup','Machine')
	link = Link(doc,group,'Machine_')
	group.Group = (bedgroup,xgroup,zgroup)
	group.ViewObject.Visibility = False
	FreeCAD.Console.PrintMessage('>>>>Machine Group End\n')
	return (link,bedgroup,xgroup,zgroup,ygroup,gimbalgroup,bed,xbase,ybase,zbase,frame,gimbal,indexergroup,indexer,trace,head,beam)

class Machine:
	def __init__(self,doc):
		FreeCAD.Console.PrintMessage('>>>>Machine Start\n')
		self.doc = doc
		self.machine,self.xgroup,self.ygroup = None,None,None
		self.machine,self.bedgroup,self.xgroup,self.zgroup,self.ygroup,self.gimbalgroup,self.bed,self.xbase,self.ybase,self.zbase,self.gframe,self.gimbal,self.indexergroup,self.indexer,self.trace,self.head,self.beam = MachineGroup(doc) 
		self.mg2l,self.ml2g,self.mvalid = Matrix(),Matrix(),True
		self.totrace,self.tracepoints = False,[]
		self.dummytrace = [Vector(0,0,-53),Vector(0,0,-53.001)]
		self.trace.Points = self.dummytrace
		self.beamlength = -self.beam.LinkedObject.Points[1].z
		FreeCAD.Console.PrintMessage('>>>>Machine End\n')
	@property
	def G2L(self):
		if self.mvalid: return self.mg2l
		self.ml2g = self.xgroup.Placement.Matrix*self.ygroup.Placement.Matrix*self.gimbalgroup.Placement.Matrix*self.indexergroup.Placement.Matrix
		self.mg2l = self.ml2g.inverse()
		self.mvalid = True
		return self.mg2l
	@property
	def AxesInfo(self):
		return (('X',-200,200),('Y',-200,200),('Z',0,350),('B',-90,90),('C',-180,180))
	@property
	def BeamLength(self):
		return self.beamlength
	@BeamLength.setter
	def BeamLength(self, value):
		if (value is not None) and (not np.isnan(value)) and (value != self.beamlength):
			self.beamlength = value
			self.beam.LinkedObject.Points = [Vector(0,0,0),Vector(0,0,-value)]
			self.doc.recompute()
	#BeamLength = property(beamlength_get,beamlength_set)
	def BeamLengthSet(self, value):
		self.BeamLength = value
	@property
	def X(self):
		return -self.xgroup.Placement.Base.x
	@X.setter
	def X(self,value):
		if (value is not None) and (not np.isnan(value)) and (value != self.X):
			self.xgroup.Placement.Base.x = -value
			self.xgroup.recompute() 
			self.mvalid = False
	@property
	def Y(self):
		return -self.ygroup.Placement.Base.y
	@Y.setter
	def Y(self,value):
		if (value is not None) and (not np.isnan(value)) and (value != self.Y):
			self.ygroup.Placement.Base.y = -value
			self.ygroup.recompute() 
			self.mvalid = False
	@property
	def Z(self):
		return self.zgroup.Placement.Base.z
	@Z.setter
	def Z(self,value):
		if (value is not None) and (not np.isnan(value)) and (value != self.Z):
			self.zgroup.Placement.Base.z = value
			self.zgroup.recompute() 
			self.mvalid = False
	@property
	def B(self):
		return -self.gimbalgroup.Placement.Rotation.Angle/math.pi*180
	@B.setter
	def B(self,value):
		if (value is not None) and (not np.isnan(value)) and (value != self.B):
			self.gimbalgroup.Placement.Rotation.Angle = -value*math.pi/180
			self.gimbalgroup.recompute() 
			self.mvalid = False
	@property
	def C(self):
		return self.indexergroup.Placement.Rotation.Angle/math.pi*180
	@C.setter
	def C(self,value):
		if (value is not None) and (not np.isnan(value)) and (value != self.C):
			self.indexergroup.Placement.Rotation.Angle = value*math.pi/180
			self.indexergroup.recompute() 	
			self.mvalid = False
	@property
	def XYZBC(self):
		return (self.X,self.Y,self.Z,self.B,self.C,self.BeamLength)
	@XYZBC.setter
	def XYZBC(self, value):
		while len(value)<6: value.append(None)
		self.X,self.Y,self.Z,self.B,self.C,self.BeamLength = value
		if self.totrace: self.Trace()
	def setTrace(self, value):
		self.totrace = value
	def resetTrace(self):
		self.tracepoints = []
		self.trace.Points = self.dummytrace
		self.doc.recompute()
#		self.trace.ViewObject.Visibility = False
	def Trace(self):
		v = self.G2L.multiply(Vector(0,0,self.Z-self.beamlength))
		self.tracepoints.append(v)
		if len(self.tracepoints)>=2:
			self.trace.Points = self.tracepoints
			self.trace.Closed = False
			self.doc.recompute()
	def GetKinematicMatricesByPrincipal(self, b, c):
		b, c = b*math.pi/180, c*math.pi/180
		sb, cb, sc, cc = math.sin(b), math.cos(b), math.sin(c), math.cos(c)		
		direct = np.array([[sb*cc, sb*sc, -cb, 0, 0, 0], [-sc, cc, 0, 0, 0, 0], [cb*cc, cb*sc, sb, 0, 0, self.BeamLength], [0, 0, 0, -1, 0, 180.0/2], [0, 0, 0, 0, -1, math.copysign(180.0, c)], [0, 0, 0, 0, 0, 1]], dtype=float)
		feedback = np.array([[sb*cc, -sc, cb*cc, 0, 0, -self.BeamLength*cb*cc], [sb*sc, cc, cb*sc, 0, 0, -self.BeamLength*cb*sc], [-cb, 0, sb, 0, 0, -self.BeamLength*sb], [0, 0, 0, -1, 0, 180.0/2], [0, 0, 0, 0, -1, math.copysign(180.0, c)], [0, 0, 0, 0, 0, 1]], dtype=float)
		return direct, feedback
	def GetKinematicMatricesByMachine(self, b, c):
		b, c = b*math.pi/180, c*math.pi/180
		sb, cb, sc, cc = math.sin(b), math.cos(b), math.sin(c), math.cos(c)		
		direct = np.array([[-cb*cc, cb*sc, -sb, 0], [-sc, -cc, 0, 0], [-sb*cc, sb*sc, cb, self.BeamLength], [0, 0, 0, 1]], dtype=float)
		feedback = np.array([[-cb*cc, -sc, -sb*cc, self.BeamLength*sb*cc], [cb*sc, -cc, sb*sc, -self.BeamLength*sb*sc], [-sb, 0, cb, -self.BeamLength*cb], [0, 0, 0, 1]], dtype=float)
		return direct, feedback
		

class MachineGui(QtGui.QMainWindow):
	def __init__(self, machine):
		FreeCAD.Console.PrintMessage('>>>>Machine GUI Start\n')
		super(MachineGui, self).__init__()
		self.machine = machine
		self.initUI()
		self.run = False
		self.runAxes = [True, True, True, True, True]
		self.runIncr = 0.2
		self.XYZBC = self.machine.XYZBC
		self.directory = '.'
	def initUI(self):
		self.result = 'Cancelled'
		# define window		xLoc,yLoc,xDim,yDim
		self.setGeometry(10, 30, 250, 315)
		self.setWindowTitle("ACS Stage v5")
		self.setWindowFlags(QtCore.Qt.WindowStaysOnTopHint)
#		grid = QtGui.QGridLayout(self)
		self.beamlabel = QtGui.QLabel('Beam length',self)
		self.beamlabel.setGeometry(15,5,60,20)
		self.beambox = QtGui.QLineEdit(f'{float(self.machine.BeamLength):8.3f}',self)
		self.beambox.setGeometry(75,5,70,20)
		self.beambox.textEdited.connect(lambda s: self.machine.BeamLengthSet(float(self.beambox.text())))
		self.labels,self.sliders,self.boxes,self.checks = [],[],[],[]
		for ax in self.machine.AxesInfo:
			i = len(self.labels)
			self.labels.append(QtGui.QLabel(ax[0],self))
			self.labels[i].setGeometry(15,30+25*i,30,20)
			self.sliders.append(QtGui.QSlider(QtCore.Qt.Horizontal,self))
			self.sliders[i].setGeometry(35,30+25*i,100,20)
			self.sliders[i].setMinimum(ax[1])
			self.sliders[i].setMaximum(ax[2])
			self.sliders[i].tag = i
			self.sliders[i].sliderMoved.connect(self.sliderMove)
			self.boxes.append(QtGui.QLineEdit('',self))
			self.boxes[i].setGeometry(145,30+25*i,70,20)
			self.boxes[i].textEdited.connect(self.axisEdited)
			self.boxes[i].tag = i
			self.checks.append(QtGui.QCheckBox(ax[0],self))
			self.checks[i].setGeometry(100+25*i,195,40,15)
			self.checks[i].setChecked(True)
			self.checks[i].stateChanged.connect(self.selectAxis)
			self.checks[i].tag = i
		self.zeroall = QtGui.QPushButton("Zero",self)
		self.zeroall.setGeometry(15,160,70,30)
		self.zeroall.clicked.connect(self.ZeroAll)
		self.animate = QtGui.QPushButton("Animate",self)
		self.animate.setGeometry(15,195,70,30)
		self.animate.clicked.connect(self.Animate)
		self.ratel = QtGui.QLabel('Rate',self)
		self.ratel.setGeometry(100,210,40,20)
		self.rates = QtGui.QSlider(QtCore.Qt.Horizontal,self)
		self.rates.setGeometry(145,210,55,20)
		self.rates.setMinimum(0)
		self.rates.setMaximum(100)
		self.rates.valueChanged.connect(self.rateMove)
		self.pstart = QtGui.QPushButton("Start",self)
		self.pstart.setGeometry(15,230,70,30)
		self.pstart.clicked.connect(self.Execute)
		self.pbox = QtGui.QLineEdit('',self)
		self.pbox.setGeometry(90,235,95,20)
		self.pbrowse = QtGui.QPushButton("Browse",self)
		self.pbrowse.setGeometry(190,235,50,20)
		self.pbrowse.clicked.connect(self.FileSelect)
		self.trace = QtGui.QCheckBox('Trace',self)
		self.trace.setGeometry(15,265,50,20)
		self.trace.stateChanged.connect(lambda s: self.machine.setTrace(self.trace.isChecked()))
		self.cleartrace = QtGui.QPushButton("Clear trace",self)
		self.cleartrace.setGeometry(65,265,70,20)
		self.cleartrace.clicked.connect(lambda s: self.machine.resetTrace())
		self.bclose = QtGui.QPushButton("Close",self)
		self.bclose.setGeometry(190,285,50,20)
		self.bclose.clicked.connect(lambda s: self.close())
		self.show()
	def closeEvent(self,event):
		App.closeDocument('AcsStage')
		mw=FreeCADGui.getMainWindow()
		mw.showMaximized()
		event.accept()
	def SetMachineAxis(self,i,v):
		pos = [None,None,None,None,None,None]
		pos[i] = v
		self.machine.XYZBC = pos
	def sliderMove(self):
		i = self.sender().tag
		v = self.sliders[i].value()
		self.boxes[i].setText(f'{float(v):8.3f}')
		self.SetMachineAxis(i,v)
	def axisEdited(self):
		i = self.sender().tag
		v = float(self.boxes[i].text())
		self.sliders[i].setValue(v)
		self.SetMachineAxis(i,v)
	@property
	def XYZBC(self):
		v = [s.value for s in self.sliders]
		v.append(float(self.beambox.getText()))
		return v
	@XYZBC.setter
	def XYZBC(self,value):
		for v,s,b in zip(value,self.sliders,self.boxes):
			s.setValue(v)
			b.setText(f'{float(v):8.3f}')
		if len(value)>5 and (value[5] is not None) and (not np.isnan(value[5])):
			self.beambox.setText(f'{float(value[5]):8.3f}')
	@property
	def Run(self):
		return self.run
	@Run.setter
	def Run(self,value):
		self.run = value
		self.animate.setText('Stop' if self.run else 'Animate')
		self.animate.setStyleSheet('color: red;' if self.run else 'color: black;')
		self.pstart.setText('Stop' if self.run else 'Start')
		self.pstart.setStyleSheet('color: red;' if self.run else 'color: black;')
	def ZeroAll(self):
		self.XYZBC = (0,0,0,0,0,None)
		self.machine.XYZBC = (0,0,0,0,0,None)
	def rateMove(self):
		self.runIncr = 0.2+self.rates.value()*0.01*1.8
	def selectAxis(self):
		i = self.sender().tag
		self.runAxes[i] = self.checks[i].isChecked()
	def AnimationStep(self,value,incr,min_,max_):
		if value<=min_:
			incr = 1
		elif value>=max_:
			incr = -1
		return (value+incr*self.runIncr,incr)
	def Animate(self):
		self.Run = not self.Run
		if not self.Run: return
		value = list(self.machine.XYZBC)
		incr = [1,1,1,1,1]
		while self.Run:
			for i,a in zip(range(5), self.machine.AxesInfo):
				if (self.runAxes[i]):
					value[i],incr[i] = self.AnimationStep(value[i],incr[i],a[1],a[2])
			self.XYZBC = value
			self.machine.XYZBC = value
			FreeCAD.Gui.updateGui()
			time.sleep(0.01)
	def FileSelect(self):
		filename, filter = QtGui.QFileDialog.getOpenFileName(parent=self, caption='Open file', dir=self.directory, filter='CSV file (*.csv)')
		FreeCAD.Console.PrintMessage(filename+'\n')
		if filename: 
			self.directory = os.path.dirname(filename)
			self.pbox.setText(os.path.basename(filename))
	def Spherical2projections(self, spheric):
		theta, phi = spheric*math.pi/180
		st, ct, sp, cp = math.sin(theta), math.cos(theta), math.sin(phi), math.cos(phi)
		return [ct*cp, ct*sp, st]
	def ToPlusMinusPi(self, a):
		return a if (a <= 180) and (a >= -180) else a + 360 if a < 0 else a - 360
	def Projections2spherical(self, proj, expectedphi):
		print('proj:', proj)
		d = np.linalg.norm(proj)
		dx, dy, dz = proj / d
		if dz > 0.9999999:
		    return 90.0, expectedphi
		else:
			theta, phi = math.asin(dz)*180.0/math.pi, math.atan2(dy, dx)*180.0/math.pi
			print('spher0:', theta, phi, expectedphi)
			if not np.isnan(expectedphi) and abs(self.ToPlusMinusPi(phi - expectedphi)) > 90: 
				theta = 180 - theta
				phi = phi + 180 if phi <= 0 else phi - 180
				print('spher1:', theta, phi)
			return theta, phi
	def GetFixtureMatrices(self, offs):
		g = 52
		x, y, z, a, b, c = offs[0], offs[1], offs[2], offs[3]*math.pi/180, offs[4]*math.pi/180, offs[5]*math.pi/180
		sa, ca, sb, cb, sc, cc = math.sin(a), math.cos(a), math.sin(b), math.cos(b), math.sin(c), math.cos(c)		
		direct = np.array([[cb*cc, -cb*sc, sb, x], [ca*sc+sa*sb*cc, ca*cc-sa*sb*sc, -sa*cb, y], [sa*sc-ca*sb*cc, sa*cc+ca*sb*sc, ca*cb, z-g], [0, 0, 0, 1]], dtype=float)
		feedback = np.array([[cb*cc, ca*sc+sa*sb*cc, sa*sc-ca*sb*cc], [-cb*sc, ca*cc-sa*sb*sc, sa*cc+ca*sb*sc], [sb, -sa*cb, ca*cb]], dtype=float)
		feedback = np.append(feedback, -feedback @ [[x],[y],[z-g]], 1)
		feedback = np.append(feedback, [[0.0, 0.0, 0.0, 1.0]], 0)
		return direct, feedback
	def Execute(self):
		self.Run = not self.Run
		if not self.Run: return
		with open(self.directory+'\\'+self.pbox.text(),'r',newline='') as f:
			row = f.readline()
			coor = 'machine'
			fixture, kinematic, ffixture, fkinematic = np.identity(4), np.identity(6), np.identity(4), np.identity(6)
			desc = None
			flen = self.machine.BeamLength
			offs = [0, 0, 0, 0, 0, 0]
			principal1, principal2, principal3 = None, None, None
			if row.startswith('{'):
				desc = json.loads(row)
				coor = desc['coordinates']
				row = f.readline()
			if coor != 'machine':
				flen = desc['focallength']
			else:
				flen = 52
			if (coor == 'part') or (coor == 'mixed'):
				print(">>>>Part coordinates")
				offs = desc['offsets']
				print('offsets:', offs)
				fixture, ffixture = self.GetFixtureMatrices(offs)
				print('fixture:', fixture)
				print('ffixture:', ffixture)
				print('verify:', fixture @ ffixture)
				#input()
				self.machine.BeamLength = flen
			while row:
				if not self.Run: return
				els = row.replace(',','\t').split('\t')
				elsnp = np.array(els, dtype=float)
				if (len(elsnp) > 5): flen = elsnp[5]
				vpart = np.append(elsnp[:5], 1.0)
				print('********************vpart:', vpart)
				vprincipal, vmachine = vpart, vpart
				if coor == 'part':
					dx, dy, dz = self.Spherical2projections(vpart[3:5])
					print('dx/dy/dz:', dx, dy, dz)
					t = fixture @ np.array([[vpart[0], dx], [vpart[1], dy], [vpart[2], dz], [1.0, 0.0]])
					theta, phi = self.Projections2spherical(t[:3,1], np.NaN if principal1 is None else principal1[4]) 
					vprincipal = np.append(t[:3,0], [theta, phi, 1.0])
					principal3, principal2, principal1  = principal2, principal1, vprincipal
					kinematic, fkinematic = self.machine.GetKinematicMatricesByPrincipal(vprincipal[3], vprincipal[4])
					vmachine = kinematic @ vprincipal
					fprincipal = fkinematic @ vmachine
					dx, dy, dz = self.Spherical2projections(np.array([fprincipal[3], fprincipal[4]]))
					t = ffixture @ np.array([[fprincipal[0], dx], [fprincipal[1], dy], [fprincipal[2], dz], [1.0, 0.0]])
					theta, phi = self.Projections2spherical(t[:3,1], vpart[4]) 
					fpart = np.append(t[:3,0], [theta, phi, 1.0])
				elif coor == 'principal':
					kinematic, fkinematic = self.machine.GetKinematicMatricesByPrincipal(vprincipal[3], vprincipal[4])
					vmachine = kinematic @ vprincipal
					fprincipal = fkinematic @ vmachine
					fpart = fprincipal
				elif coor == 'mixed':
					vprincipal = fixture @ np.array([vpart[0], vpart[1], vpart[2], 1.0])
					#print('principal:', vprincipal)
					b, c = -vpart[3]-offs[4], vpart[4]-offs[5]
					kinematic, fkinematic = self.machine.GetKinematicMatricesByMachine(b, c)
					vmachine = kinematic @ vprincipal
					fprincipal = fkinematic @ vmachine
					fpart = ffixture @ fprincipal
					vmachine = np.append(vmachine[:3], [b, c, 1.0])
					fprincipal = np.append(fprincipal[:3], [vpart[3], vpart[4], 1.0])
					fpart = np.append(fpart[:3], [vpart[3], vpart[4], 1.0])
				elif coor == 'machine':
					vmachine = vpart
					vmachine[0] = -vmachine[0]
					vmachine[1] = -vmachine[1]
					vmachine[3] = -vmachine[3]
					fpart = vpart
				dvec = vpart - fpart
				print('fpart:', fpart)
				print('difference:', dvec)
				if any(d > 1.e-6 for d in dvec): print('@@@@@@@@@@@@@@@@@@@@@@@@@@')
				vec = np.append(vmachine[:5], flen)
				self.XYZBC = vec
				self.machine.XYZBC = vec
				FreeCAD.Gui.updateGui()
				time.sleep(0.01)
				row = f.readline()
		self.Run = False

doc = App.newDocument('AcsStage')
machine = Machine(doc)
doc.recompute()
Gui.SendMsgToActiveView("ViewFit")
Gui.activeDocument().activeView().viewIsometric()
machine.indexer.ViewObject.ShapeColor = (1.00,0.83,0.14)
machine.gimbal.ViewObject.ShapeColor = (0.79,1.00,0.04)
machine.head.ViewObject.ShapeColor = (84./255,193./255,1.)
machine.ybase.ViewObject.ShapeColor = (1.,1.,127./255)
machine.gframe.ViewObject.ShapeColor = (1.,170./255,1.)
machine.trace.ViewObject.LineColor = (1.,0.,0.)
machine.trace.ViewObject.PointColor = (1.,0.,0.)
machine.beam.ViewObject.LineColor = (1.,1.,1.)
FreeCAD.Gui.updateGui()
mw=FreeCADGui.getMainWindow()
ev=QtGui.QKeyEvent(QtCore.QEvent.KeyPress,QtCore.Qt.Key_F11,QtCore.Qt.NoModifier)
QtGui.QApplication.sendEvent(mw,ev)
mw.showMinimized()
gui = MachineGui(machine)



