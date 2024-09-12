import os
import sys
import machine
import engine_main
import engine
import engine_io
import engine_draw
from engine_math import Vector3
import time

sys.path.append("/Games/ThumbyRaster")
os.chdir("/Games/ThumbyRaster")

from ThumbyRasterMath import *
import ThumbyRaster

machine.freq(300 * 1000 * 1000)

def Main():
    
    gc.collect()
    LastTimeMS = time.ticks_ms()
    MainRenderState = RenderState()
    MeshFileName = "Peaches_Castle.bin"
    MaterialFileName = "Peaches_Castle_global_mat.bin"
    #MeshFileName = "bbb_room.bin"
    #MaterialFileName = "bbb_room_global_mat.bin"
    #MeshFileName = "dragon.bin"
    #MaterialFileName = "dragon_global_mat.bin"
    TestModel = Model()
    TestModel.SetAlphaClipState(True, Vector3(1.0, 1.0, 1.0))
    TestModel.LoadModel(MeshFileName, MaterialFileName, True)
    
    tris = [[1.0, 1.0, -1.0, 1.0, 0.0, 0.0],
            [1.0, 2.0, -1.0, 0.0, 1.0, 0.0],
            [2.0, 2.0, -1.0, 0.0, 0.0, 1.0],
            [2.0, 2.0, -1.0, 0.0, 0.0, 1.0],
            [2.0, 1.0, -1.0, 0.0, 1.0, 0.0],
            [1.0, 1.0, -1.0, 1.0, 0.0, 0.0]]
    addtris = [[2.0, 2.0, -1.0, 1.0, 0.0, 0.0],
            [2.0, 3.0, -1.0, 0.0, 1.0, 0.0],
            [3.0, 3.0, -1.0, 0.0, 0.0, 0.0],
            [3.0, 3.0, -1.0, 0.0, 0.0, 0.0],
            [3.0, 2.0, -1.0, 0.0, 1.0, 0.0],
            [2.0, 2.0, -1.0, 1.0, 0.0, 0.0]]
    TestDynamicModel = Model()
    TestDynamicModel.SetDynamicMeshTriangles(tris)
    TestDynamicModel.AddDynamicMeshTriangles(addtris)
    TestDynamicModel.SetUseVertexColor(True)
    gc.collect()

    ObjectRot = Matrix44()
    ObjectRot.Identity()
    #ObjectRot.Euler(45.0, 0.0, 45.0)
    ObjectPos = Matrix44()
    ObjectPos.Identity()
    #ObjectPos.Translation(Vector3(0.0, 0.0, -10.0))
    ObjectTF = ObjectPos.Mul(ObjectRot)
    TestModel.SetTransform(ObjectTF)
    TestDynamicModel.SetTransform(ObjectTF)

    CameraAngle = 180.0
    Roll = 0.0
    Yaw = CameraAngle
    Pitch = 0.0
    
    CameraPosition = Vector3(0.0, -1.0, 0.0)
    
    while(True):
        if engine.tick():
            MovementScale = 0.3
            RotationScale = 2.0
            Forward = 0.0
            Strafe = 0.0
            Up = 0.0
            if engine_io.LEFT.is_pressed:
                CameraAngle -= RotationScale
            if engine_io.RIGHT.is_pressed:
                CameraAngle += RotationScale
            if engine_io.DOWN.is_pressed:
                Forward += MovementScale
            if engine_io.UP.is_pressed:
                Forward -= MovementScale
            if engine_io.A.is_pressed:
                Up -= MovementScale
            if engine_io.B.is_pressed:
                Up += MovementScale
            if engine_io.LB.is_pressed:
                Strafe -= MovementScale
            if engine_io.RB.is_pressed:
                Strafe += MovementScale
                
                
            CurrentTimeMS = time.ticks_ms()
            TimeDiffMS = CurrentTimeMS - LastTimeMS
            print(1000.0 / TimeDiffMS)
            LastTimeMS = time.ticks_ms()

                            
            ForwardVector = AngleToForward(CameraAngle)
            RightVector = AngleToRight(CameraAngle)
            UpVector = AngleToUp(CameraAngle)
            
            CameraPosition = Vector3AddV(Vector3MulF(ForwardVector, Forward), CameraPosition)
            CameraPosition = Vector3AddV(Vector3MulF(RightVector, Strafe), CameraPosition)
            CameraPosition = Vector3AddV(Vector3MulF(UpVector, Up), CameraPosition)

            DisplayBuffer = engine_draw.back_fb_data()
            MainRenderState.SetRenderTarget(DisplayBuffer)
            MainRenderState.ClearColor(Vector3(0.0, 0.2, 0.2))
            # Non-Reverse Depth
            MainRenderState.ClearDepth(1000.0)
            
            Yaw = CameraAngle
            ViewRotation = Matrix44()
            ViewRotation.Euler(Roll, Yaw, Pitch)
            ViewTranslation = Matrix44()
            ViewTranslation.Translation(CameraPosition)
            ViewMatrix = ViewRotation.Mul(ViewTranslation)
            MainRenderState.SetViewMatrix(ViewMatrix)

            # Render state can be used to render different viewports
            TestModel.Draw(MainRenderState)
            TestDynamicModel.Draw(MainRenderState)
            # Physics test against world model
            # Sphere against tris       
    
Main()



