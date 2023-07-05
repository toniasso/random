# -*- coding: mbcs -*-
from part import *
from material import *
from section import *
from optimization import *
from assembly import *
from step import *
from interaction import *
from load import *
from mesh import *
from job import *
from sketch import *
from visualization import *
from connectorBehavior import *
import numpy as np



m=4
n=4
VF = 0.02
size = 1
width = 1
wire = 1
positions = np.arange(0.5, max(m, n)+1, width)
displacementU1=0.01
displacementU2=0.01

seedMatrix = 0.04
seedGraphene = 0.02

ModelName = 'Matrix' + str(m) + 'x' + str(n)

angles = np.multiply( np.random.random( (m, n) ), 360 )

#MODEL
mdb.Model(modelType=STANDARD_EXPLICIT, name=ModelName)

#REFERENCE POINT
mdb.models[ModelName].rootAssembly.ReferencePoint(point=(m*1.1, -n*0.1, 0.0))

mdb.models[ModelName].rootAssembly.Set(name='RF', referencePoints=(
        mdb.models[ModelName].rootAssembly.referencePoints[1], ))


#MATERIALS
mdb.models[ModelName].Material(name='AluminumMaterial')
mdb.models[ModelName].materials['AluminumMaterial'].Elastic(table=((75000.0,
        0.36), ))
mdb.models[ ModelName ].Material( name='GrapheneMaterial' )
mdb.models[ ModelName ].materials[ 'GrapheneMaterial' ].Elastic( table=((1000000, 0.2),) )

mdb.models[ModelName].HomogeneousSolidSection(material='AluminumMaterial',
        name='MatrixSection', thickness=None)

#MATRIX
mdb.models[ModelName].ConstrainedSketch(name='__profile__', sheetSize=10.0)
mdb.models[ModelName].sketches['__profile__'].rectangle(point1=(0.0, 0.0),
        point2=(m, n))
mdb.models[ModelName].Part(dimensionality=THREE_D, name='MatrixPart', type=
        DEFORMABLE_BODY)
mdb.models[ModelName].parts['MatrixPart'].BaseShell(sketch=
        mdb.models[ModelName].sketches['__profile__'])
del mdb.models[ModelName].sketches['__profile__']
mdb.models[ModelName].parts['MatrixPart'].setValues(space=TWO_D_PLANAR, type=
        DEFORMABLE_BODY)
mdb.models[ModelName].parts['MatrixPart'].SectionAssignment(offset=0.0,
        offsetField='', offsetType=MIDDLE_SURFACE, region=Region(
        faces=mdb.models[ModelName].parts['MatrixPart'].faces.getSequenceFromMask(
        mask=('[#1 ]', ), )), sectionName='MatrixSection', thicknessAssignment=
        FROM_SECTION)
mdb.models[ModelName].parts['MatrixPart'].seedPart( deviationFactor=0.1,
                                                    minSizeFactor=0.1, size=seedMatrix )
mdb.models[ModelName].parts['MatrixPart'].generateMesh()

#GRAPHENE
mdb.models[ModelName].ConstrainedSketch(name='__profile__', sheetSize=10.0)
mdb.models[ModelName].sketches['__profile__'].Line(point1=(0.0, -0.25), point2=
        (0.0, 0.25))
mdb.models[ModelName].Part(dimensionality=TWO_D_PLANAR, name='GraphenePart', type=
DEFORMABLE_BODY)
mdb.models[ModelName].parts['GraphenePart'].BaseWire(sketch=
        mdb.models[ModelName].sketches['__profile__'])
del mdb.models[ModelName].sketches['__profile__']
depth = wire
height = float(VF * size * size * width / wire * wire)
mdb.models[ModelName].RectangularProfile(a=height, b=depth, name='BeamProfile')
mdb.models[ModelName].BeamSection(consistentMassMatrix=False, integration=
        DURING_ANALYSIS, material='GrapheneMaterial', name='GrapheneSection',
        poissonRatio=0.0, profile='BeamProfile', temperatureVar=LINEAR)
mdb.models[ModelName].parts['GraphenePart'].SectionAssignment(offset=0.0,
        offsetField='', offsetType=MIDDLE_SURFACE, region=Region(
        edges=mdb.models[ModelName].parts['GraphenePart'].edges.getSequenceFromMask(
        mask=('[#1 ]', ), )), sectionName='GrapheneSection', thicknessAssignment=
        FROM_SECTION)
mdb.models[ModelName].parts['GraphenePart'].seedPart(deviationFactor=0.1,
        minSizeFactor=0.1, size=0.05)
mdb.models[ModelName].parts['GraphenePart'].Set(edges=
            mdb.models[ModelName].parts['GraphenePart'].edges.getSequenceFromMask((
                '[#1 ]',), ), name='VigaPropertySet')
mdb.models[ModelName].parts['GraphenePart'].assignBeamSectionOrientation(method=N1_COSINES, n1=[0.0, 1.0, 0.0], region=
                                                                                    mdb.models[ModelName].parts[
                                                                                        'GraphenePart'].sets[
                                                                                        'VigaPropertySet'])
mdb.models[ModelName].parts['GraphenePart'].generateMesh()

#STEP
mdb.models[ModelName].StaticStep(name='Step-1', previous='Initial')

#ASSEMBLY
mdb.models[ModelName].rootAssembly.DatumCsysByDefault(CARTESIAN)
mdb.models[ModelName].rootAssembly.Instance(dependent=ON, name='MatrixPart-1',
        part=mdb.models[ModelName].parts['MatrixPart'])

a=0
b=0
for i in range(m):
    b=0
    for j in range(n):
        nome=' a: ' + str(a) + ' b: '+ str(b)
        mdb.models[ModelName].rootAssembly.Instance(dependent=ON, name='GrapheneAssembly' + nome
                , part=mdb.models[ModelName].parts['GraphenePart'])
        mdb.models[ModelName].rootAssembly.translate(instanceList=('GrapheneAssembly' + nome, ),
                vector=(positions[a], positions[b], 0.0))
        mdb.models[ModelName].rootAssembly.rotate(angle=angles[a,b], axisDirection=(0.0, 0.0,
                1.0), axisPoint=(positions[a], positions[b], 0.0), instanceList=('GrapheneAssembly' + nome, ))
        b = b + 1
    a = a + 1

# EMBEDDED REGION
a=0
b=0
for i in range(m):
    b = 0
    for j in range(n):
        nome = ' a: ' + str( a ) + ' b: ' + str( b )
        mdb.models[ModelName].EmbeddedRegion(absoluteTolerance=0.0, embeddedRegion=
                Region(
                edges=mdb.models[ModelName].rootAssembly.instances['GrapheneAssembly' + nome].edges.getSequenceFromMask(
                mask=('[#1 ]', ), )), fractionalTolerance=0.05, hostRegion=None, name=
                'Embedded' + nome, toleranceMethod=BOTH, weightFactorTolerance=1e-06)
        b = b + 1
    a = a + 1

#BOUNDARY CONDITIONS
mdb.models[ModelName].DisplacementBC(amplitude=UNSET, createStepName=
            'Step-1', distributionType=UNIFORM, fieldName='', fixed=OFF, localCsys=None
                                                 , name='HorizontalDisplacement', region=Region(
                    edges=mdb.models[ModelName].rootAssembly.instances['MatrixPart-1'].edges.getSequenceFromMask(
                        mask=('[#8 ]',), )), u1=0.0, u2=UNSET, ur3=UNSET)
mdb.models[ModelName].DisplacementBC(amplitude=UNSET, createStepName='Step-1',
        distributionType=UNIFORM, fieldName='', localCsys=None, name='VerticalDisplacement',
        region=Region(
        edges=mdb.models[ModelName].rootAssembly.instances['MatrixPart-1'].edges.getSequenceFromMask(
        mask=('[#4 ]', ), )), u1=UNSET, u2=SET, ur3=UNSET)

#SETS
mdb.models[ModelName].parts['MatrixPart'].Set(edges=mdb.models[ModelName].parts['MatrixPart'].edges.getSequenceFromMask(('[#2 ]', ), ), name='Matrix_X')
mdb.models[ModelName].parts['MatrixPart'].Set(edges=mdb.models[ModelName].parts['MatrixPart'].edges.getSequenceFromMask(('[#1 ]', ), ), name='Matrix_Y')
mdb.models[ModelName].rootAssembly.regenerate()

#EQUATIONS
mdb.models[ModelName].Equation(name='X_Const', terms=((1.0,
        'MatrixPart-1.Matrix_X', 1), (-1.0, 'RF', 1)))
mdb.models[ModelName].Equation(name='Y_Const', terms=((1.0,'MatrixPart-1.Matrix_Y', 1), (1.0, 'RF', 1)))
mdb.models[ModelName].Equation(name='Y_Const', terms=((1.0,
        'MatrixPart-1.Matrix_Y', 2), (1.0, 'RF', 2)))


#DISPLACEMENT
mdb.models[ ModelName ].DisplacementBC( amplitude=UNSET, createStepName='Step-1',
                                            distributionType=UNIFORM, fieldName='', fixed=OFF, localCsys=None, name=
                                            'Displacement', region=mdb.models[ ModelName ].rootAssembly.sets[ 'RF' ],
                                            u1=displacementU1, u2=displacementU2, u3=UNSET )

mdb.models[ModelName].HistoryOutputRequest(createStepName='Step-1', name=
        'H-Output-2', rebar=EXCLUDE, region=
        mdb.models[ModelName].rootAssembly.sets['RF'], sectionPoints=DEFAULT,
        variables=('RF1', 'U1'))

#JOB
JobName='Job_' + str(m) + 'x' + str(n)
mdb.Job( atTime=None, contactPrint=OFF, description='', echoPrint=OFF,
                 explicitPrecision=SINGLE, getMemoryFromAnalysis=True, historyPrint=OFF,
                 memory=90, memoryUnits=PERCENTAGE, model=ModelName, modelPrint=OFF,
                 multiprocessingMode=DEFAULT, name=JobName, nodalOutputPrecision=SINGLE,
                 numCpus=1, numGPUs=0, queue=None, resultsFormat=ODB, scratch='', type=
                 ANALYSIS, userSubroutine='', waitHours=0, waitMinutes=0 )
mdb.jobs[ JobName ].submit( consistencyChecking=OFF )


