# -*- coding: utf-8 -*-
"""
Created on Sat Jun 27 13:40:16 2020

@author: sachchit
"""

import time
from datetime import datetime
import pytz
import cryptography
from Cryptodome.PublicKey import RSA
from Cryptodome.Hash import SHA256
from Cryptodome.Signature import pkcs1_15
import signxml as sx
from lxml import etree
import logging
import jks
import base64, textwrap
import threading
import json
import socket
import sys
from os import listdir
from os import path

class NPNT():
    def __init__(self):
        self.flightLlogFolder = "./NPNT/Flight_Logs/"
        self.paFolder = "./NPNT/Permission_Artefact/"
        self.keyStoreFolder = "./NPNT/Key_Store/"
        
        self.keystore = jks.KeyStore.load(self.keyStoreFolder+"Keystore.jks", "Sam")
        
        # NPNT Status
        self.__npntAllowed = False
        self.__npntNotAllowedReason = b''
        
        # PA Related
        self.paVerified = False
        self.paUploaded = False
        self.permissionArtefactFileName = ''
        self.permissionArtefactTree = None
        self.permissionArtefactTreeRoot = None
        self.timeZone = pytz.timezone('Asia/Kolkata')
        
        # Logging
        self.takeOffPointLat = -200
        self.takeOffPointLon = -200
        self.takeOffTimeStamp = 0
        self.landPointLat = -200
        self.landPointLon = -200
        self.landTimeStamp = 0
        self.breached = False
        self.tookOff = False
        self.landed = True
        self.breachedLat = []
        self.breachedLon = []
        self.breachedTimeStamp = []
        self.permissionArtefactId = None
        self.fileIndex = 0
        
        # Lock
        self.lock = threading.Lock()
        
        # RFM Server
        self.rfmServer = RFMServer(self.flightLlogFolder, self.paFolder, self.keyStoreFolder)
        
        # Variable to kill all threads cleanly
        self.killAllThread = threading.Event()
    
    def get_npnt_allowed(self):
        return int(self.__npntAllowed)
        
    def get_npnt_not_allowed_reason(self):
        return self.__npntNotAllowedReason
    
    def parse_permission_artefact(self):
        self.permissionArtefactTree = etree.parse(self.paFolder + self.permissionArtefactFileName)
        self.permissionArtefactTreeRoot = self.permissionArtefactTree.getroot()
        
        # Read Permission Artifact ID
        self.permissionArtefactId = self.permissionArtefactTreeRoot.get("permissionArtifactId")
        
        #read Coordinates from PA   
        self.fence = Fence()
        coordinates = self.permissionArtefactTreeRoot.find(".//Coordinates")
        for x in coordinates.findall('Coordinate'):
            self.fence.add_point((float(x.get('latitude')),float(x.get('longitude'))))
            
        #read time from PA
        self.flightStartTime = 0
        self.flightEndTime = 0
        flightParams = self.permissionArtefactTreeRoot.find(".//FlightParameters")
        startTime = flightParams.get('flightStartTime')[0:19].replace("T", " ")
        startTimeZone = flightParams.get('flightStartTime')[-6:]
        endTime = flightParams.get('flightEndTime')[0:19].replace("T", " ")
        endTimeZone = flightParams.get('flightEndTime')[-6:]
        if startTimeZone != "+05:30" and endTimeZone != "+05:30":
            logging.info('NPNT, Timezone issue')
            return False
        
        self.flightStartTime = self.timeZone.localize(datetime.strptime(startTime, "%Y-%m-%d %H:%M:%S"), is_dst=None)
        self.flightEndTime = self.timeZone.localize(datetime.strptime(endTime, "%Y-%m-%d %H:%M:%S"), is_dst=None)

        return True
    
    def verify_xml_signature(self):
        """
        Verify the signature of a given xml file against a certificate
        :param path xml_file: path to the xml file for verification
        :return: bool: the success of verification
        """
        certificate = self.read_cert()
        
        try:
            sx.XMLVerifier().verify(data=self.permissionArtefactTreeRoot, require_x509=True, x509_cert=certificate).signed_xml
            # The file signature is authentic
            return True
        except cryptography.exceptions.InvalidSignature:
            # add the type of exception
            return False
        
    def read_cert(self):
        for alias, c in self.keystore.certs.items():
            return ("\r\n".join(textwrap.wrap(base64.b64encode(c.cert).decode('ascii'), 64)))
    
    def read_key(self):
        for alias, pk in self.keystore.private_keys.items():
            s=("-----BEGIN RSA PRIVATE KEY-----\n")
            s+=("\r\n".join(textwrap.wrap(base64.b64encode(pk.pkey).decode('ascii'), 64)))
            s+=("\n-----END RSA PRIVATE KEY-----")
            return ''.join(s)
    
    def check_permission_artifact(self, lat, lon, hdop):
        if len(self.permissionArtefactFileName) == 0:
            return False
        
        # check Lat Lon data from autopilot are correct
        if lat > -190e7 and lon > -190e7 and hdop < 2:
            # First Parse the data
            if not self.parse_permission_artefact():
                self.__npntAllowed = False
                self.__npntNotAllowedReason = b'NPNT PA Parsing Error'
                logging.info("NPNT, PA Parsing Error")
                return False
            
            #Check if PA is Valid and verify signature
            if not self.verify_xml_signature():
                self.__npntAllowed = False
                self.__npntNotAllowedReason = b'NPNT Signature Mismatch'
                logging.info("NPNT, PA Signature Mismatch")
                return False
            
            return True
        else:
            self.__npntAllowed = False
            self.__npntNotAllowedReason = b'NPNT no GPS Lock'
            logging.info("NPNT, GPS Lock is not available")
            return False
    
    def within_time(self, timestamp):
        if timestamp > 0:
            currentTime = pytz.utc.localize(datetime.utcfromtimestamp(timestamp)).astimezone(self.timeZone)
            if self.flightStartTime<=currentTime and currentTime<=self.flightEndTime:
                return True
            else:
                logging.info("NPNT, Outside Time Limit")
                return False
        else:
            logging.info("NPNT, Not getting time from autopilot")
            return False
        
    def write_log(self):
        with self.lock:
            # Convert Data to proper format for json export
            geoFenceBreach = []
            for i in range(len(self.breachedLat)):
                geoFenceBreach.append({"Latitude":self.breachedLat[i],
                                       "Longitude":self.breachedLon[i],
                                       "TimeStamp":self.breachedTimeStamp[i]
                                       })
            landData = {"Latitude":self.landPointLat,
                        "Longitude":self.landPointLon,
                        "TimeStamp":self.landTimeStamp
                        }
            
            takeOffData = {"Latitude":self.takeOffPointLat,
                           "Longitude":self.takeOffPointLon,
                           "TimeStamp":self.takeOffTimeStamp
                           }
            
            self.takeOffPointLat = -200
            self.takeOffPointLon = -200
            self.takeOffTimeStamp = 0
            self.landPointLat = -200
            self.landPointLon = -200
            self.landTimeStamp = 0
            self.breachedLat = []
            self.breachedLon = []
            self.breachedTimeStamp = []
            
        # Creating dictionary for flight log
        flightLog = {"FlightLog": {"GeofenceBreach": geoFenceBreach,
                                   "Land": landData,
                                   "TakeOff":takeOffData
                                   }
            }
        # Creating json data for flight log
#        flightLogJD = json.dumps(logDataUnsigned,  indent=4)
        
        # Adding Permission Artifact Information for log
        flightLog["PermissionArtefact"] = self.permissionArtefactId
        
        # Signing Flight Log
        rsaKey = RSA.import_key(self.read_key())
        hashedLogData = SHA256.new(json.dumps((flightLog["FlightLog"])).encode())
        logSignature = pkcs1_15.new(rsaKey).sign(hashedLogData)
        # the signature is encoded in base64 for transport
        enc = base64.b64encode(logSignature)
        # dealing with python's byte string expression
        flightLog['Signature'] = enc.decode('ascii')
        
        # Creating file name
        logFileName = self.flightLlogFolder+"signed_log_" + self.permissionArtefactId + "_" + str(self.fileIndex) + "_" + ".json"
        self.fileIndex = self.fileIndex + 1
        
        with open(logFileName, "w") as signedLogFile:
            json.dump(flightLog, signedLogFile, indent=4)
        
        logging.info("NPNT, Signed log file written")
        
    def update(self, lat, lon, hdop, globalTime, isArmed):
        if self.paUploaded:
            if (self.permissionArtefactFileName != self.rfmServer.paFilename):
                self.permissionArtefactFileName = self.rfmServer.paFilename
                logging.info("%s"%self.permissionArtefactFileName)
                self.paVerified = False
            if self.paVerified:
                if isArmed:
                    #If we are entering this place first time record take-off
                    if not self.tookOff:
                        logging.info("NPNT, TakeOff Point Recorded")
                        self.takeOffPointLat = lat*1e-7
                        self.takeOffPointLon = lon*1e-7
                        self.takeOffTimeStamp = globalTime
                        self.tookOff = True
                        self.landed = False
                
                    # Handle Breach
                    if not self.fence.check_point((lat*1e-7,lon*1e-7)): 
                        logging.info("NPNT, Currently Breaching the Geofence")
                        self.breached = True
                        
                    if not self.within_time(globalTime):
                        logging.info("NPNT, Currently Breaching the Time Limit")
                        self.breached = True
                    
                    if self.breached:
                        self.breachedLat.append(lat*1e-7)
                        self.breachedLon.append(lon*1e-7)
                        self.breachedTimeStamp.append(globalTime)
                    
                else:
                    #If we are entering this place after landing record the landing point
                    if not self.landed:
                        logging.info("NPNT, Landing Point Recorded")
                        self.landPointLat = lat*1e-7
                        self.landPointLon = lon*1e-7
                        self.landTimeStamp = globalTime
                        self.breached = False
                        self.tookOff = False
                        self.landed = True
                        logWriteThread = threading.Thread(target=self.write_log)
                        logWriteThread.daemon = True
                        logWriteThread.start()


                    if self.fence.check_point((lat*1e-7,lon*1e-7)):
                        if self.within_time(globalTime):
                            #allow to arm
                            self.__npntAllowed = True
                            self.__npntNotAllowedReason = b'Go Go Go'
                            logging.info("NPNT, Allowing to Arm")
                        else:
                            #Don't allow arming
                            self.__npntAllowed = False
                            self.__npntNotAllowedReason = b'NPNT Time Breach'
                            pass
                    else:
                        #Don't allow arming
                        self.__npntAllowed = False
                        self.__npntNotAllowedReason = b'NPNT GeoFence Breach'
                        logging.info("NPNT, Outside GeoFence")
            else:
                if self.check_permission_artifact(lat, lon, hdop):
                    self.paVerified = True
                    logging.info('NPNT, PA Verified')
                else:
                    self.paVerified = False
        else:
            logging.info('NPNT, PA Not Uploaded')
            self.__npntAllowed = False
            self.__npntNotAllowedReason = b'NPNT PA not Uploaded'
            if not self.rfmServer.started:
                self.rfmServer.start_server()
            if self.rfmServer.paAvailable:
                self.permissionArtefactFileName = self.rfmServer.paFilename
                self.paUploaded = True
                
    def kill_all_threads(self):
        logging.info("NPNT killing all threads")
        self.rfmServer.kill_all_threads()
        self.killAllThread.set()
        logging.info("NPNT joined all threads")
                

class RFMServer:
    def __init__(self, flightLogFolder, paFolder, keyFolder):
        self.started = False
        
        # Connection related
        #self.serverAddress = ('192.168.168.1', 7000)
        self.serverAddress = ('127.0.0.1', 7000)
        self.connection = None
        self.sock = None
        
        #public key related global variables
        self.publicKeyFolder = keyFolder
        self.publicKeyFilename = 'test_key.pem'
        
        #flight log related global variables
        self.flightLogFolder = flightLogFolder
        self.fltLogReqest = False # make it false again after associated event is triggered
        self.fltLogName = ''
        self.fltLogFlag = False # make it false again after associated event is triggered
        
        #permission artifact related global variables
        self.paFolder = paFolder
        self.paFilename = ''
        self.paString = '' #permission artifact string
        self.paAvailable = False  # make it false again after associated event is triggered
        self.paFilenameFlag = False
        self.paFileflag = False
        
        self.rfmServerThread = None
        
        # Variable to kill all threads cleanly
        self.killAllThread = threading.Event()
        
    # Read permission artifact and save to file as well as store in a global variable
    # Flag g_PA_available is provided to know if ermission artifact is available to access or not, this flag shoud be checked continuosly
    def check_and_save_pa_received_from_client(self, strdata):
    
        if(strdata.__contains__('$PA_START$') or self.paFileflag):
            if(strdata.__contains__('$PA_START$')):
                self.paString = ''
                self.paFileflag = True
    
            if(self.paFileflag):
                self.paString = self.paString + strdata
                if(strdata.__contains__('$PA_END$')):
                    self.paFileflag = False
                    tok = self.paString.split('$PA_START$')
                    if(len(tok)>1):
                        tok = tok[1].split('$PA_END$')
                        if(len(tok)>1):
                            tok = tok[0]
                            tok = tok.replace('$PA_FILENAME_START$','$PA_FILENAME_END$')
                            tok = tok.split('$PA_FILENAME_END$')
                            if(len(tok)>2):
                                self.paFilename = tok[1]
                                self.paString = tok[0] + tok[2]
                                with open(self.paFolder+self.paFilename,'w') as pafile:
                                    pafile.write(self.paString)
                                self.send_status_to_client("Permission artifact receival acknowledged")
                                self.paAvailable = True
    
    def check_flight_log_name_request_from_client(self, strdata):
        if(strdata.__contains__('$REF_LOGS$')):
            self.send_flight_log_file_names_to_client()
    
    def send_flight_log_file_names_to_client(self):
        if(path.isdir(self.flightLogFolder)):
            logList = listdir(self.flightLogFolder)
            if(len(logList)>0):
                logString = ','.join(logList)
                self.connection.sendall(('$LOG_NAMES_START$'+logString+'$LOG_NAMES_END$').encode())
                self.send_status_to_client('Flight log file names request acknowledged')
            else:
                self.send_status_to_client('No log files available')
        else:
            self.send_status_to_client('Request error')
    
    def check_flight_log_request_from_client(self, strdata):
        if(strdata.__contains__('$REQ_LOG_START$')):
            self.fltLogFlag = True
        if(self.fltLogFlag):
            self.fltLogName = self.fltLogName + strdata
        if(self.fltLogFlag and strdata.__contains__('$REQ_LOG_END$')):
            self.fltLogName = self.fltLogName.replace('$REQ_LOG_START$','')
            self.fltLogName = self.fltLogName.replace('$REQ_LOG_END$','')
            self.send_flight_log_to_client(self.fltLogName)
            self.fltLogFlag = False
            self.fltLogName = ''
    
    def send_flight_log_to_client(self, fltLogName):
        if(path.isdir(self.flightLogFolder)):
            with open(self.flightLogFolder + fltLogName ,'r') as file:
                filedata = file.read()
            self.send_status_to_client(fltLogName + '- file download request acknowledged')
            self.connection.sendall(('$LOG_START$'+filedata+'$LOG_END$').encode())
        else:
            self.send_status_to_client('Resuest Error')
    
    def check_public_key_request_from_client(self, strdata):
        if(strdata.__contains__('$REQ_KEY$')):
            if(path.isdir(self.publicKeyFolder)):
                if(path.isfile(self.publicKeyFolder + self.publicKeyFilename)):
                    with open(self.publicKeyFolder + self.publicKeyFilename,'r') as file:
                        filedata = file.read()
                self.send_status_to_client('Public key request acknowledged')
                self.send_public_key_to_client(filedata)
            else:
                self.send_status_to_client('Resuest Error')
    
    
    def send_public_key_to_client(self, key):
        self.connection.sendall(('$KEY_START$'+key+'$KEY_END$').encode())
    
    def send_status_to_client(self, strdata):
        self.connection.sendall(('>>'+strdata+ '<<').encode())
    
    def start_server(self):
        self.rfmServerThread = threading.Thread(target=self.socket_init)
        self.rfmServerThread.start()
    
    def socket_init(self):
        self.started = True
        
        # Create a TCP/IP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    
        # Bind the socket to the port
        logging.info("RFM Server, Starting up on %s port %s" % self.serverAddress )
        self.sock.bind(self.serverAddress)
    
        # Listen for incoming connections
        self.sock.listen(1)
    
        while True:
            # Wait for a connection
            logging.info("RFM Server, Waiting for a connection")
            self.connection, clientAddress = self.sock.accept()
            
            if self.killAllThread.is_set():
                break
            
            try:
                logging.info("RFM Server, Connection from %s"%(str(clientAddress)))
                # Receive the data in small chunks
                while True:
                    if self.killAllThread.is_set():
                        break
                    data = self.connection.recv(1024)
    
                    if data:
                        self.check_and_save_pa_received_from_client(data.decode())
                        self.check_flight_log_name_request_from_client(data.decode())
                        self.check_public_key_request_from_client(data.decode())
                        self.check_flight_log_request_from_client(data.decode())
                    else:
                        logging.info("RFM Server, No more data from %s"%(str(clientAddress)))
                        break
            finally:
                # Clean up the connection
                self.connection.close()
    
    def kill_all_threads(self):
        logging.info("RFM Server killing all threads")
        self.killAllThread.set()
        if self.connection is None:
            self.sock.close()
        else:
            self.connection.close()
        self.rfmServerThread.join()
        logging.info("RFM Server joined all threads")

# Class to define a fence.
class Fence:
    """
    A Fence object is primarily a way of containing a list of points that form
    a boundary, and providing the ability to add / change points in the fence.
    """

    points = None

    def __init__(self):
        self.points = []

    def add_point(self, point):
        # Add new point to points list
        self.points.append(point)

    def list_points(self):
        return(self.points)

    def check_point(self, point, debug = False):
        """
        check_point()  checks if a given point lies inside a given fence polygon.
        Parameters are given as an instance of the fence class and a given point as
        a, tuple of (x, y).
        """
        def check_in_bounds(point, line_eqns_index):
            """
            check_in_bounds() checks if a supplied point is within the upper and
            lower x & y bounds of the points that form a given line.

            Takes the point to be checked and the index that points to which line
            is being tested. Knowing the location of the line equation in its list
            we can find the points that it is formed from.

            Returns true if the point is within bound and false if it's outside
            bounds.
            """
            withinX = False;
            withinY = False;

            pointA = self.points[line_eqns_index]
            # If the index is pointing to the last member of the list, the line
            # is made of points that wrap around back to the start of the points
            # list.
            if line_eqns_index + 1 == len(self.points):
                pointB = self.points[0]
            else:
                pointB = self.points[line_eqns_index + 1]

#            print(point)
#            print(pointA, pointB)
            # Check if point[x] is within pointA[x] and pointB[x].
            if (pointA[0] >= pointB[0]):
                # pointA is more positive than pointB so check if point[x] is
                # between these.
                if (point[0] <= pointA[0] and point[0] >= pointB[0]):
                    withinX = True
            elif (pointA[0] <= pointB[0]):
                # pointA is less positive than pointB.
                if (point[0] >= pointA[0] and point[0] <= pointB[0]):
                    withinX = True

            # Check if point[y] is within pointA[y] and pointB[y].
            if (pointA[1] >= pointB[1]):
#                print("pointA[1] >= pointB[1]")
                # pointA is more positive than pointB so check if point[y] is
                # between these.
                if (point[1] <= pointA[1] and point[1] >= pointB[1]):
                    withinY = True
            elif (pointA[1] <= pointB[1]):
#                print("pointA[1] <= pointB[1]")
                # pointA is less positive than pointB.
                if (point[1] >= pointA[1] and point[1] <= pointB[1]):
                    withinY = True

#            print(withinX, withinY)

            if withinX and withinY:
                return True
            else:
                return False

        # Find numbe of intersections of the fence with the point horizon line on
        # either side of the point.
        def find_intersect(line, point_horizon_eqn):
            """
            find_intersect() returns a coordinate if an intersection exists, and
            False if not.

            Uses Cramers rule to work it out.
            """
            # Calculate the determinant.
            D  = line[0] * point_horizon_eqn[1] - line[1] * point_horizon_eqn[0]
            Dx = line[2] * point_horizon_eqn[1] - line[1] * point_horizon_eqn[2]
            Dy = line[0] * point_horizon_eqn[2] - line[2] * point_horizon_eqn[0]
            if D != 0:
                x = Dx / D
                y = Dy / D
                return (x, y)
            else:
                return False

        # First, find the horozontal line equation that passes through the point.
        # Using the Ax + By = C fomat, express as a tuple of (A, B, C).
        point_horizon_eqn = (0, 1, point[1])

        # Next, form equations with the list of points given by the Fence object.
        if len(self.points) < 3:
            raise Exception("The supplied fence has not enough (< 3) points!")
        # Form list of line equations. Last in list will circle back to first point.
        # E.g. self.points = [(A), (B), (C)], line_eqns = [(AB), (BC), (CA)].
        # Where each element in the line_eqns list is a tuple of (m, c).
        line_eqns = []
        for point_index in range(len(self.points)):
            point1 = self.points[point_index]
            # If point1 is the last in the list, point2 is the first element of the
            # list.
            if point_index == (len(self.points) - 1):
                point2 = self.points[0]
            else:
                point2 = self.points[point_index + 1]
            # Check if vertical or horizontal line first.
            if point1[1] == point2[1]:
                # Hoizontal
                a = 0
                b = 1
                c = point1[1]
#                if debug == True:
#                    print("Formed equation", str(a) + "x+" + str(b) + "y=" + str(c), "from points:", point1, "and", point2)
                line_eqns.append((a, b, c))
            elif point1[0] == point2[0]:
                # Vertical
                a = 1
                b = 0
                c = point1[0]
#                if debug == True:
#                    print("Formed equation", str(a) + "x+" + str(b) + "y=" + str(c), "from points:", point1, "and", point2)
                line_eqns.append((a, b, c))
            else:
                # Non vertical or hoizontal line.
                a = (-1 * ((point2[1] - point1[1]) / (point2[0] - point1[0])))
                b = 1
                # c = y - ax
                c = (point1[1] + (a * point1[0]))
#                if debug == True:
#                    print("Formed equation", str(a) + "x+" + str(b) + "y=" + str(c), "from points:", point1, "and", point2)
                line_eqns.append((a, b, c))

#        if debug == True:
#            print("\n")
#            print("All equations formed (in order):", line_eqns)
#            print("Finding intersections...\n")
#            print("\nx bounds are:", str(self.max_x), str(self.min_x), "y bounds:", str(self.max_y), str(self.min_y))
        intersection_points_left = []
        intersection_points_right = []
        for line_index in range(0, len(line_eqns)):
            intersection_point = find_intersect(line_eqns[line_index], point_horizon_eqn)
#            if debug == True:
#                print("Intersection point between lines:", line_eqns[line_index], "&", point_horizon_eqn, "is:", intersection_point)
            if intersection_point != False:
                if intersection_point[0] < point[0]:
                    # Intersection point x value is less than point x value.
                    if (intersection_point not in intersection_points_left) and check_in_bounds(intersection_point, line_index) == True:
                        intersection_points_left.append(intersection_point)
                else:
                    # Intersection point x value is greater than point x value.
                    if (intersection_point not in intersection_points_right) and check_in_bounds(intersection_point, line_index) == True:
                        intersection_points_right.append(intersection_point)

        # Check if the number of intersections to the left and right are odd.
        if len(intersection_points_left) > 0 and len(intersection_points_right) > 0:
#            if debug == True:
#                print("\n")
#                print((str(len(intersection_points_left))), "intersection points to the left.")
#                print((str(len(intersection_points_right))), "intersection points to the right.")
            if ((len(intersection_points_left) % 2) == 1) and ((len(intersection_points_right) % 2) == 1):
                # Both have odd intersection counts, so the point is in the Fence.
                return(True)
            else:
#                print(intersection_points_left, intersection_points_right)
                return(False)
        else:
#            print(intersection_points_left, intersection_points_right)
            return(False)

def convertDMSToDD(degrees, minutes, seconds):
    return(degrees + (minutes / 60) + (seconds / 3600))

   
def fence_coordinates(g_fence):
    n = len(g_fence)
    my_fence = Fence()
    for i in range(0,n):
        my_fence.add_point((g_fence[i][0],g_fence[i][1]))
        
    return my_fence

def drone_inside_fence(lat,lon,my_fence):
    geofencce_output = my_fence.check_point((lat,lon))
    
    return geofencce_output


#npnt=NPNT()
#npnt.parse_permission_artefact()
