#!/usr/bin/env python


import roslib
roslib.load_manifest('crazyflieROS')
import rosbag
import rospy
import sys
import numpy as np
import scipy.io as sio
from tf import Transformer



def printInfo(bag, topics=None):
    # get interesting lines of information
    p = False
    lines = []
    info = str(bag).splitlines()
    for line in info:
        if line.startswith("topics"):
            p = True

        if not p:
            continue
        line = line[len("topics:      "):]
        lines.append(line)

    # simplify string
    lines = [[p for p in l.split(" ") if len(p)>0] for l in lines]

    # Only keep relevant topics
    if topics is None:
        lines = [l[0:8] for l in lines if len(l)>7]
    else:
        lines = [l[0:8] for l in lines if l[0] in topics]

    if lines == []:
        return None, None
    topicNameLen = max(4, max([len(l[0]) for l in lines]))
    topicTypeLen = max(8, max([len(l[7]) for l in lines]))

    # Print information
    print
    for l in info[0:7]:
        print l
    s = "| %-"+str(topicNameLen)+"s | %-7s | %-9s | %-"+str(topicTypeLen)+"s |"
    h = s.replace(" ", "")
    print s % ("Topic", "Number", "Frequency", "Msg Type")
    print h % ((2+topicNameLen)*"-", 9*"-", 11*"-", (2+topicTypeLen)*"-")
    for l in lines:
        print s % (l[0], l[1], str(l[4])+" "+str(l[5]), l[7])

    t0 = info[3]
    t1 = info[4]

    t0 = rospy.Time(float(t0[t0.find('(')+1:t0.rfind(')')]))
    t1 = rospy.Time(float(t1[t1.find('(')+1:t1.rfind(')')]))
    return t0, t1 #return duration



def process(bagfile):

    # Load bag
    print "\nOpening bag file [%s]" % bagfile
    bag = rosbag.Bag(bagfile)


    
    hz = 100

    # Get dictionary of topics
    # topics = list(set([c.topic for c in bag._get_connections() if c.topic.startswith("/cf") or c.topic.]))
    topics = list(set([c.topic for c in bag._get_connections() if c.topic != "/diagnostics" and c.topic.rfind("rosout")+c.topic.rfind("parameter_") == -2]))

    if topics == []:
        rospy.logwarn('[%s] has no topics, skipping')
        return
    start, finish = printInfo(bag, topics=topics)

    dur = finish-start 
    rostf = Transformer(True, dur) #buffer
    duration = dur.to_sec()



    # deal with TF
    # treat each tf frame as a topic
    # tfdict = {}
    # for tfs in [m[1] for m in bag.read_messages("/tf")]:
    #     #from->to
    #     for t in tfs.transforms:
    #         tFrom = t.header.frame_id
    #         tTo   = t.child_frame_id
    #         if not tfdict.has_key(tFrom):
    #             tfdict[tFrom] = {}
    #         if not tfdict[tFrom].has_key(tTo):
    #             tfdict[tFrom][tTo] = []
    #         tfdict[tFrom][tTo].append(t)




    
    for tfs in bag.read_messages("/tf"):
        for t in tfs[1].transforms:
            t.header.stamp -= start

            if (t.header.stamp < rospy.Duration(0)):
                rospy.logwarn('%s - %s [%f]' % (t.header.frame_id, t.child_frame_id, t.header.stamp.to_sec()))
                continue

            rostf.setTransform(t) #cache all tf info into messages to a transformer object

            # tFrom = t.header.frame_id
            # tTo   = t.child_frame_id
            # if not tfdict.has_key(tTo):
            #     tfdict[tTo] = {}
            # if not tfdict[tTo].has_key(tFrom):
            #     tfdict[tTo][tFrom] = []
            # tfdict[tTo][tFrom].append(t)



    # # Check
    # for f in tfdict.keys():
    #     fs = tfdict[f].keys()
    #     if (len(fs)>1):
    #         rospy.logerr("Warning: Frame [%s] has multiple parent frames: %s", f, fs)
    # for f in sorted(tfdict.keys()):
    #     for fs in sorted(tfdict[f].keys()):
    #         print fs, "->", f
    # print "___"


    # Print existing frames
    # get [(child_frame, parent_frame), ...] for all known transforms in bag
    framestr = [(k[k.rfind(' ')+1:],k[0:k.find(' ')]) for k in rostf.allFramesAsString().replace('Frame ','').split('.\n') if k]
    for parent, child in framestr:
        if parent != "NO_PARENT":
            print parent, "->", child


    # List of transforms we are interested in using
    FRAMES = [("/world", "/cf_gt"), ("/world", "/goal"), ("/cf_gt2d", "/goal")]  # from->to pairs

    # For TF we interpolate at equal intervals
    timelist = np.arange(0, duration, 1./hz)

    # container for all transforms
    frames_pq = {"time": timelist}
    for parent, child in FRAMES:
        # Get TF info at given HZ
        pq = []
        for ti in timelist:
            t = rospy.Time(secs=ti)
            if rostf.canTransform(parent, child, t):
                p, q = rostf.lookupTransform(parent, child, t)
                pq.append([p[0],p[1],p[2],q[0],q[1],q[2],q[3]])
            else:
                #rospy.logwarn("Could not get transform %s -> %s at time %fs", parent, child, t.to_sec())
                pq.append([0,0,0, 0,0,0,1])
        frames_pq[parent[1:].replace('/','_')+"__"+child[1:].replace('/','_')] = np.array(pq)  # matlab friendly


    # # PLot xyz
    # import matplotlib.pyplot as plt
    # # Create the plot
    # plt.plot(timelist, frames_pq["world-cf_gt"][:, 0:3]) # FRAME, time step, dimension
    # plt.plot(timelist, frames_pq["world-goal"][ :, 0:3]) # FRAME, time step, dimension
    # plt.show()

    # Collect all data
    data = {t: {} for t in topics if t != '/tf' and not t.endswith('image_raw') and not t.endswith('camera_info')}

    # Prepare all the dictionaries
    for t in data.keys():
        print t
        name, msg, t = bag.read_messages(topics=t).next()        
        data[name] = {slot: [] for slot in msg.__slots__ if slot != "header"}
        data[name]["time"] = []


    # print "USED DATA: \n",
    [k +": "+str(data[k].keys()) for k in data.keys()]


    # Fill all dictionaries
    for name, msg, t in bag.read_messages(topics=data.keys()):
        # They all have a time stamp
        data[name]["time"].append((t-start).to_sec())
        # Append data for each slot
        for slot in data[name].keys():
            if slot == "time":
                continue
            contents = msg.__getattribute__(slot)
            data[name][slot].append(contents)

    # Convert all to NP arrays
    mdata = {}
    for name in data.keys():
        mname = name[name.rfind('/')+1:]  # matlab friendly name
        mdata[mname] = {}
        for slot in data[name].keys():
            mdata[mname][slot] = np.array(data[name][slot])


    # add TF data
    mdata['tf'] = frames_pq

    # Save as MAT
    sio.savemat(bagfile[:-4]+'.mat', {'bag'+bagfile[:-4].replace('-','_'):mdata}, oned_as='column')

    return


   
def main():    
    bagfile = sys.argv[1]

    if bagfile=='-a':
        import glob
        import os
        os.chdir("./")
        for f in glob.glob("*.bag"):
            process(f)
    else:
        process(bagfile)

        

    return


           
if __name__ == "__main__":
    main()
