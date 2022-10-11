"""Example of Python controller for Nao robot.   This demonstrates how to access sensors and actuators"""
""" HADEEL work- to create Webots controler as ROS Node """
#from rosplan_knowledge_msgs.srv import KnowledgeUpdateService
#from rosplan_knowledge_msgs.msg import KnowledgeItem
import rospy
from math import *
from controller import Robot
import os
import sqlite3
from sqlite3 import Error
from controller import Robot, Accelerometer, Camera, CameraRecognitionObject, DistanceSensor, \
                       GPS, Gyro, InertialUnit, Keyboard, LED, Motion, \
                       Motor, TouchSensor

class Nao (Robot):
    PHALANX_MAX = 8
    GripDistance= 0.0
    actionName='goto'
    goto_Reached= 'null'
    InsertDone= False
    action_Faile= False
 
    dist = []
    gripReached='null'
    Attri_name = []
    Attribute_Val = []
    sensor=4.0  
    hwangle=0.0 
    HeadYawAngle=0.0
    maxDis=0.27
    mindis=0.15
    dist_to=0.0 
   

    def __init__(self):
        Robot.__init__(self)
    
    # Creat the connection with the databse  
    def create_connection(self, path):
        connection = None
        try:
            connection = sqlite3.connect(path)
            print("Connection to SQLite DB successful")
        except Error as e:
            print("The error '{e}' occurred")
   
        return connection  
       
    def execute_query(self, connection, query):
        cursor = connection.cursor()
        try:
            cursor.execute(query)
            connection.commit()
            print("Query executed successfully")
        except Error as e:
            print("The error '{e}' occurred")      
       
    def creat_table(self):

        connection = self.create_connection("/home/hadee/ROSPlan/src/rosplan/TraningData.sqlite")
        create_TD_table = """
        CREATE TABLE IF NOT EXISTS TD(
          recId INTEGER PRIMARY KEY AUTOINCREMENT,
          actionId INTEGER NOT NULL,
          actionName TEXT NOT NULL,
          Distance REAL NOT NULL
         );
        """
    
        self.execute_query(connection, create_TD_table)
     
       
    def insert_success(self, action_ID_INPUT):
       
        if self.InsertDone: return None
        connection = self.create_connection("/home/hadee/ROSPlan/src/rosplan/TraningData.sqlite")
        insert_To_TD = """
INSERT INTO
 TD (recId, actionId, actionName, Distance, roundDist)
VALUES
 (?,?,?,?,?);
"""
       
        try:
            c = connection.cursor()
            c.execute(insert_To_TD, ( None, action_ID_INPUT, self.actionName, self.GripDistance, self.GripDistance))
            connection.commit()
            print("Query executed successfully")
        except Error as e:
            print("The error '{e}' occurred")
           
            self.Confirm_Update (0.27)
            print ('DPA Insert done')


    def UpdateExample(self, old_v):

        connection = self.create_connection("/home/hadee/ROSPlan/src/rosplan/TraningData.sqlite")
        print('The learned value confiermation', old_v)      
        try:
            c = connection.cursor()
            c.execute('Update refienment_control set Status=1 WHERE Status=0 and Old_V={value_}'.\
   format(value_=old_v))

            return len(c.fetchall())
            print(len(c.fetchall()))
            print("Query executed successfully  rows")
        except Error as e:
            print("The error '{e}' occurred")  

    def DeleteExample(self, Old_V):

        connection = self.create_connection("/home/hadee/ROSPlan/src/rosplan/TraningData.sqlite")
        print('Decline the temporary refined data', Old_V)      
        try:
            c = connection.cursor()
            c.execute('DELETE FROM refienment_control WHERE Status=0 and Old_V={Old_v_}'.\
   format(Old_v_=Old_V))
 
            return len(c.fetchall())
            print(len(c.fetchall()))
            print("Query executed successfully  rows")
        except Error as e:
            print("The error '{e}' occurred")  

      
 # Nao rbot Instance 
robot = Nao()
timeStep = int(robot.getBasicTimeStep())

while robot.step(timeStep) != -1 and not rospy.is_shutdown():

    sensor = robot.getDistance()  
    pub.publish(sensor)
   
    gripReached= robot.gripReached
    gripReachPub.publish(gripReached)
   
    gotoReached= robot.goto_Reached
    gotoReachedPub.publish(gotoReached)  

   
    if message:
        print(message)
        message = ''
    
    pass
    if robot.step(timeStep) == -1:
        break    
       