import os
import sys
import machine
import engine_main
import engine
import engine_io
import engine_draw
from engine_math import Vector3
#import random
#import time

sys.path.append("/Games/ThumbyRaster")
os.chdir("/Games/ThumbyRaster")

from ThumbyRasterMath import *
import ThumbyRaster

machine.freq(300 * 1000 * 1000)

def Main():
    MainRenderState = RenderState()
    #meshfilename = "Peaches_Castle.bin"
    #materialfilename = "Peaches_Castle_global_mat.bin"
    #MeshFileName = "bbb_room.bin"
    #MaterialFileName = "bbb_room_global_mat.bin"
    MeshFileName = "dragon.bin"
    MaterialFileName = "dragon_global_mat.bin"
    TestModel = Model()
    TestModel.LoadModel(MeshFileName, MaterialFileName)

    CameraAngle = 180.0
    Roll = 0.0
    Yaw = CameraAngle
    Pitch = 0.0
    
    CameraPosition = Vector3(6.0, -6.0, -12.0)
    
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
            
            # Physics test against world model
            # Sphere against tris       
    
Main()



