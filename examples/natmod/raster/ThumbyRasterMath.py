from engine_math import Vector3
import struct
import gc
import math
import array
import sys
import rp2

import ThumbyRaster

# Temporary Engine Vector3 math functions
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
            [1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0])
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
    def Identity(self):
        self.Data = array.array('f',
            [1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0])
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
        self.MeshColor = Vector3(1.0, 1.0, 1.0)
        self.NumVerts = 0
        self.NumShapes = 0
        self.NumMaterials = 0
        self.TotalPixels = 0
        self.Vertices = array.array('f')
        self.ShapeMaterialMap = array.array('I')
        self.GlobalMaterialMap = array.array('I')
        self.GlobalMaterialData = array.array('H')
        self.ObjectTransform = Matrix44()
        self.ObjectTransform.Identity()
        self.AlphaMaskColor = Vector3(1.0, 1.0, 1.0)
        self.EnableAlphaMask = False
        self.UseMeshColor = False
        self.UseVertexColor = False
        self.IsDynamicMesh = False
        self.__ScratchData = bytearray([])
        self.__StoreInScratch = False
        self.__ResourceAddress = 0
        
    def __CheckFlush(self):
        if(len(self.__ScratchData) == 4096):
            print("checkflush")
            print(str(len(self.__ScratchData)))
            GlobalBlockAllocator.WriteScratch(self.__ScratchData)
            self.__ScratchData = bytearray([])
            gc.collect()
    
    def __ForceFlush(self):
        if(len(self.__ScratchData) > 0):
            print("forceflush")
            print(str(len(self.__ScratchData)))
            GlobalBlockAllocator.WriteScratch(self.__ScratchData)
            self.__ScratchData = bytearray([])
            gc.collect()
    
    def __LoadMesh(self, MeshFileName):
        MeshFile = open(MeshFileName, "rb")
        if(self.__StoreInScratch):
            self.__MeshResourceAddress = GlobalBlockAllocator.GetBaseAddress()
            self.NumVerts = int.from_bytes(MeshFile.read(4), "little")
            j = 0
            for i in range(self.NumVerts * 5):
                self.__ScratchData.extend(MeshFile.read(4))
                self.__CheckFlush()
                if(j % 5 == 4):
                    self.__ScratchData.extend(bytearray([0, 0, 0, 0]))
                    self.__CheckFlush()
                j += 1
                
            self.NumShapes = int.from_bytes(MeshFile.read(4), "little")
            for i in range(2 * self.NumShapes):
                self.__ScratchData.extend(MeshFile.read(4))
                self.__CheckFlush()
            self.__ForceFlush()
        else:
            MeshFile = open(MeshFileName, "rb")
            # All mesh data is dword sized   
            self.NumVerts = int.from_bytes(MeshFile.read(4), "little")
            # Load vertices with position + uv
            j = 0
            for i in range(0, self.NumVerts * 5, 1):
                VertElement = MeshFile.read(4)
                Unpackedverts = struct.unpack('<f', VertElement)[0]
                self.Vertices.append(Unpackedverts)
                if(j % 5 == 4):
                    self.Vertices.append(0.0)
                j += 1
                
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
        
    def __LoadMaterial(self, MaterialFileName):
        self.HasTexture = True
        MaterialFile = open(MaterialFileName, "rb")
        if(self.__StoreInScratch):
            self.__MaterialResourceAddress = GlobalBlockAllocator.GetBaseAddress()
            self.NumMaterials = int.from_bytes(MaterialFile.read(4), "little")
            for i in range(3 * self.NumMaterials):
                self.__ScratchData.extend(MaterialFile.read(4))
                self.__CheckFlush()
            self.TotalPixels = int.from_bytes(MaterialFile.read(4), "little")
            for i in range(self.TotalPixels):
                self.__ScratchData.extend(MaterialFile.read(4))
                self.__CheckFlush()
            self.__ForceFlush()
        else:
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
        
    def AddDynamicMeshTriangles(self, PositionArray):
        if (self.IsDynamicMesh):
            self.NumVerts += len(PositionArray)
            for Position in PositionArray:
                self.Vertices.append(Position[0])
                self.Vertices.append(Position[1])
                self.Vertices.append(Position[2])
                self.Vertices.append(Position[3])
                self.Vertices.append(Position[4])
                self.Vertices.append(Position[5])
            self.ShapeMaterialMap = array.array('I', [self.NumVerts * 10, 0])
        else:
            self.SetDynamicMeshTriangles(PositionArray)
    
    def SetDynamicMeshTriangles(self, PositionArray):
        self.NumVerts = len(PositionArray)
        self.Vertices = array.array('f', range(self.NumVerts*6))
        i = 0
        for Position in PositionArray:
            self.Vertices[i * 6 + 0] = Position[0]
            self.Vertices[i * 6 + 1] = Position[1]
            self.Vertices[i * 6 + 2] = Position[2]
            self.Vertices[i * 6 + 3] = Position[3]
            self.Vertices[i * 6 + 4] = Position[4]
            self.Vertices[i * 6 + 5] = Position[5]
            i += 1
        self.ShapeMaterialMap = array.array('I', [self.NumVerts * 10, 0])
        self.GlobalMaterialMap = array.array('I', [1, 1, 0])
        self.GlobalMaterialData = array.array('H', [0, 0, 0, 0])
        self.IsDynamicMesh = True
        
    def SetUseVertexColor(self, UseVertexColor : bool):
        self.UseVertexColor = UseVertexColor
    
    def SetMeshColor(self, UseMeshColor : bool, Color : Vector3):
        self.UseMeshColor = UseMeshColor
        self.MeshColor = Color
        
    def SetAlphaClipState(self, EnableAlphaMask : bool, Color : Vector3):
        self.EnableAlphaMask = EnableAlphaMask
        self.AlphaMaskColor = Color
        
    def SetTransform(self, Transform : Matrix44):
        self.ObjectTransform = Transform
                            
    def LoadModel(self, MeshFileName, MaterialFileName, StoreInScratch=False):
        self.__ScratchData = bytearray()
        self.__StoreInScratch = StoreInScratch
        self.__LoadMesh(MeshFileName)
        self.__LoadMaterial(MaterialFileName)

    def Draw(self, State : RenderState):
        ModelView = Matrix44()
        ViewMat = Matrix44()
        ViewMat.Data = State.ViewMatrix
        ModelView = ViewMat.Mul(self.ObjectTransform)
        BlendState = array.array('f',
            [self.EnableAlphaMask,
            self.AlphaMaskColor.x,
            self.AlphaMaskColor.y,
            self.AlphaMaskColor.z])
        ShaderState = array.array('f',
            [self.UseMeshColor,
            self.UseVertexColor,
            self.MeshColor.x,
            self.MeshColor.y,
            self.MeshColor.z])
        if(self.__StoreInScratch):
            vertexaddr = self.__MeshResourceAddress
            print(str(vertexaddr))
            shapeaddr = vertexaddr + 4 * 6 * self.NumVerts
            materialmapaddr = self.__MaterialResourceAddress
            materialdataaddr = materialmapaddr + 4 * 3 * self.NumMaterials
            readout = array.array('f', range(int(1024)))
            ThumbyRaster.DrawTrianglesTexScratch(
                BlendState,
                ShaderState,
                State.RenderTarget,
                State.DepthBuffer,
                State.CameraPosition,
                ModelView.Data,
                State.ProjMatrix,
                self.NumVerts,
                vertexaddr,
                shapeaddr,
                materialmapaddr,
                materialdataaddr)
        else:
            ThumbyRaster.DrawTrianglesTex(
                BlendState,
                ShaderState,
                State.RenderTarget,
                State.DepthBuffer,
                State.CameraPosition,
                ModelView.Data,
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
        self.ProgramSize = 1024*1024
        self.PageSize = 4096
        self.ScratchSize = 2*1024*1024
        # Two MB backwards from the start of fs memory
        self.ScratchMemoryPageStartAddress = -2 * self.ProgramSize//self.PageSize
        self.FreePageList = [range(self.ScratchSize//self.PageSize)]
        #self.AllocationMap = {}
        self.__NextPageAddress = 0
        
    def Reset(self):
        self.__NextPageAddress = 0
        
    def ReadWriteFloatArrayTest(self, TestFloatArray):
        numfloats = len(TestFloatArray)
        PageAddressStart = self.ScratchMemoryPageStartAddress
        rp2.Flash().writeblocks(PageAddressStart, TestFloatArray)
        OutFloatArray = array.array('f', [0.0, 0.0, 0.0, 0.0])
        rp2.Flash().readblocks(PageAddressStart, OutFloatArray)
        print(OutFloatArray)
        
    def GetBaseAddress(self):
        return (self.ScratchMemoryPageStartAddress + 512 + 256 + self.__NextPageAddress) * 4096 + 0x10000000
        
    def WriteScratch(self, Data : bytearray):
        PageAddressStart = self.ScratchMemoryPageStartAddress + self.__NextPageAddress
        rp2.Flash().writeblocks(PageAddressStart, Data)
        ByteAddress = (self.ScratchMemoryPageStartAddress + 512 + 256 + self.__NextPageAddress) * 4096 + 0x10000000
        RoundPage = 0
        if ((len(Data) - ((len(Data) // 4096) * 4096)) > 0):
            RoundPage = 1
        self.__NextPageAddress = self.__NextPageAddress + (len(Data) // 4096) + RoundPage
        return ByteAddress
    
    
GlobalBlockAllocator = BlockAllocator()

