from engine_math import Vector3
import struct
import gc
import math
import array
import sys
import rp2

import ThumbyRaster

# Temporary Engine Vector3 math functiosn
def Vector3AddV(A : Vector3, B : Vector3):
    return Vector3(A.x + B.x, A.y + B.y, A.z + B.z)
def Vector3MulF(A : Vector3, B : float):
    return Vector3(A.x * B, A.y * B, A.z * B)
def Vector3Dot(A : Vector3, B : Vector3):
    return A.x * B.x + A.y * B.y + A.z * B.z
# Direction vectors assuming
def AngleToForward(Angle : float):
    Rads = math.pi * Angle / 180.0
    return Vector3(-math.sin(Rads), 0.0, math.cos(Rads))
def AngleToRight(Angle : float):
    Rads = math.pi * Angle / 180.0
    return Vector3(math.cos(Rads), 0.0, math.sin(Rads))
def AngleToUp(Angle : float):
    return Vector3(0.0, 1.0, 0.0)

class Vector4:
    def __init__(self, x = 0.0, y = 0.0, z = 0.0, w = 0.0):
        self.Data = array.array('f', [x, y, z, w])
    def Dot(self, Vec : Vector4):
        DotOut = self.Data[0] * Vec.Data[0]
        DotOut += self.Data[1] * Vec.Data[1]
        DotOut += self.Data[2] * Vec.Data[2]
        DotOut += self.Data[3] * Vec.Data[3]
        return DotOut
class Matrix44:
    def __init__(self):
        self.Data = array.array('f',
            [0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0])
    def GetRow(self, Row : int):
        return Vector4(
            self.Data[Row * 4 + 0], 
            self.Data[Row * 4 + 1],
            self.Data[Row * 4 + 2],
            self.Data[Row * 4 + 3])
    def Mul(self, Mat : Matrix44):
        MatOut = Matrix44()
        ColVec0 = Vector4(Mat.Data[0], Mat.Data[4], Mat.Data[8], Mat.Data[12])
        ColVec1 = Vector4(Mat.Data[1], Mat.Data[5], Mat.Data[9], Mat.Data[13])
        ColVec2 = Vector4(Mat.Data[2], Mat.Data[6], Mat.Data[10], Mat.Data[14])
        ColVec3 = Vector4(Mat.Data[3], Mat.Data[7], Mat.Data[11], Mat.Data[15])
        for Row in range(0, 4, 1):
            MatOut.Data[Row * 4 + 0] = self.GetRow(Row).Dot(ColVec0);
            MatOut.Data[Row * 4 + 1] = self.GetRow(Row).Dot(ColVec1);
            MatOut.Data[Row * 4 + 2] = self.GetRow(Row).Dot(ColVec2);
            MatOut.Data[Row * 4 + 3] = self.GetRow(Row).Dot(ColVec3);
        return MatOut
    def Translation(self, Position : Vector3):
        self.Data = array.array('f',
            [1.0, 0.0, 0.0, Position.x,
            0.0, 1.0, 0.0, Position.y,
            0.0, 0.0, 1.0, Position.z,
            0.0, 0.0, 0.0, 1.0])
    def Euler(self, Yaw : float, Pitch : float, Roll : float):
        Alpha = Yaw * math.pi / 180.0
        Beta = Pitch * math.pi / 180.0
        Gamma = Roll * math.pi / 180.0
        self.Data[0] = math.cos(Beta) * math.cos(Gamma)
        self.Data[1] = math.sin(Alpha) * math.sin(Beta) * math.cos(Gamma) - math.cos(Alpha) * math.sin(Gamma)
        self.Data[2] = math.cos(Alpha) * math.sin(Beta) * math.cos(Gamma) + math.sin(Alpha) * math.sin(Gamma)
        self.Data[3] = 0.0
        self.Data[4] = math.cos(Beta) * math.sin(Gamma)
        self.Data[5] = math.sin(Alpha) * math.sin(Beta) * math.sin(Gamma) + math.cos(Alpha) * math.cos(Gamma)
        self.Data[6] = math.cos(Alpha) * math.sin(Beta) * math.sin(Gamma) - math.sin(Alpha) * math.cos(Gamma)
        self.Data[7] = 0.0
        self.Data[8] = -math.sin(Beta)
        self.Data[9] = math.sin(Alpha) * math.cos(Beta)
        self.Data[10] = math.cos(Alpha) * math.cos(Beta)
        self.Data[11] = 0.0
        self.Data[12] = 0.0
        self.Data[13] = 0.0
        self.Data[14] = 0.0
        self.Data[15] = 1.0
        
class RenderState:
    def __init__(self):
        self.Width = 128
        self.Height = 128
        self.RenderTarget = array.array('H')
        self.DepthBuffer = array.array('f', range(int(self.Width * self.Height)))
        self.CameraPosition = array.array('f', [0.0, 0.0, 0.0])
        Near = 0.01
        Far = 10.0
        AngleOfView = 90.0
        FOV = 1.0 / math.tan(AngleOfView * 0.5 * math.pi / 180.0)
        AspectRatio = 1.0
        self.ProjMatrix = array.array('f',
            [AspectRatio * FOV, 0.0, 0.0, 0.0,
            0.0, FOV, 0.0, 0.0,
            0.0, 0.0, Far / (Far - Near), (-Far * Near) / (Far - Near),
            0.0, 0.0, 1.0, 0.0])
        self.ViewMatrix = array.array('f',
            [1.0, 0.0, 0.0, self.CameraPosition[0],
            0.0, 1.0, 0.0, self.CameraPosition[1],
            0.0, 0.0, 1.0, self.CameraPosition[2],
            0.0, 0.0, 0.0, 1.0])
    def SetViewMatrix(self, ViewMatrix : Matrix44):
        self.ViewMatrix = ViewMatrix.Data
    def SetRenderTarget(self, RenderTarget):
        self.RenderTarget = RenderTarget              
    def ClearColor(self, ClearColorValue : Vector3):
        Color = array.array('f',
            [ClearColorValue.x,
             ClearColorValue.y,
             ClearColorValue.z])
        ThumbyRaster.ClearRenderTarget(
            self.RenderTarget,
            Color)
    def ClearDepth(self, DepthValue : float):
        Depth = array.array('f', [DepthValue])
        ThumbyRaster.ClearDepthBuffer(
            self.DepthBuffer,
            Depth)

class Model:
    def __init__(self):
        self.HasTexture = False
        self.IsDynamicMesh = False
        # rgb 8 8 8
        self.MeshColor = array.array('f', [255.0, 255.0, 255.0])
        self.NumVerts = 0
        self.NumShapes = 0
        self.NumMaterials = 0
        self.TotalPixels = 0
        self.Vertices = array.array('f')
        self.ShapeMaterialMap = array.array('I')
        self.GlobalMaterialMap = array.array('I')
        self.GlobalMaterialData = array.array('H')
        
    def LoadMesh(self, MeshFileName):
        MeshFile = open(MeshFileName, "rb")
        # All mesh data is dword sized   
        self.NumVerts = int.from_bytes(MeshFile.read(4), "little")
        # Load vertices with position + uv
        for i in range(0, self.NumVerts * 5, 1):
            VertElement = MeshFile.read(4)
            Unpackedverts = struct.unpack('<f', VertElement)[0]
            self.Vertices.append(Unpackedverts)
        # Load number of shapes
        self.NumShapes = int.from_bytes(MeshFile.read(4), "little")
        gc.collect()
        # Load shape materials
        for i in range(0, self.NumShapes * 2, 1):
            ShapeElem = MeshFile.read(4)
            Unpackedelem = struct.unpack('<I', ShapeElem)[0]
            self.ShapeMaterialMap.append(Unpackedelem)
        MeshFile.close()
        gc.collect()
        
    def LoadMaterial(self, MaterialFileName):
        self.HasTexture = True
        MaterialFile = open(MaterialFileName, "rb")
        # Load number of materials
        self.NumMaterials = int.from_bytes(MaterialFile.read(4), "little")
        for i in range(0, self.NumMaterials * 3, 1):
            MatElem = MaterialFile.read(4)
            UnpackedElem = struct.unpack('<I', MatElem)[0]
            self.GlobalMaterialMap.append(UnpackedElem)
        gc.collect()
        # Load texture atlas
        self.TotalPixels = int.from_bytes(MaterialFile.read(4), "little")
        self.GlobalMaterialData = array.array('H', range(int(self.TotalPixels)))
        for i in range(0, self.TotalPixels//2, 1):
            ColorElem = MaterialFile.read(2)
            UnpackedElem = struct.unpack('<H', ColorElem)[0]
            self.GlobalMaterialData[i] = UnpackedElem
        MaterialFile.close()
        gc.collect()
        
    def SetDynamicMeshTriangles():
        self.IsDynamicMesh = True
    
    def SetMeshColor(self, Color : Vector3):
        self.MeshColor = Color
                            
    def LoadModel(self, MeshFileName, MaterialFileName, StoreInFlash=False):
        self.LoadMesh(MeshFileName)
        self.LoadMaterial(MaterialFileName)
        
    def Draw(self, State : RenderState):
        ThumbyRaster.DrawTrianglesTex(
                State.RenderTarget,
                State.DepthBuffer,
                State.CameraPosition,
                State.ViewMatrix,
                State.ProjMatrix,
                self.NumVerts,
                self.Vertices,
                self.ShapeMaterialMap,
                self.GlobalMaterialMap,
                self.GlobalMaterialData)

# TODO Buffer and flush multiple allocations within a page
# Configurable number of pages for small block allocation
class BlockAllocator:
    def __init__(self):
        self.XIPSize = 1024*1024
        self.PageSize = 4096
        self.ScratchSize = 2*1024*1024
        self.ScratchMemoryPageStartAddress = self.XIPSize//self.PageSize
        self.FreePageList = range(self.ScratchSize//self.PageSize)
        self.AllocationMap = {}
        self.NextResourceID = 0
        print(self.FreePageList)
        
    def FindStartOfRange(NumPages : int):
        pass
        
    def WriteFirmware(self, Data):
        #SizeInPages = //4096
        
        # Allocate pages
        
        # Sort page list by page number
        self.AllocationMap.sort()
        
        #testb = bytearray(4096)
        PageAddressStart = self.ScratchMemoryPageStartAddress
        rp2.Flash().writeblocks(blockoffset, testb)
        rp2.Flash().readblocks(blockoffset, testb)
        
        # TODO Pick a unique resource ID, one that is not in the allocation map
        ResourceUID = self.NextResourceID
        self.NextResourceID += 1
        return ResourceUID
    
    # TODO Move pages to consecutive locations to make larger consecutive free blocks
    # there is no virtual addressing
    def CompactPages(self):
        pass