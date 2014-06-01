#!/usr/bin/env python


import roslib
roslib.load_manifest('crazyflieROS')
import rosbag
import rospy
import sys
import numpy as np
import scipy.io as sio



   
def main ():
    bagfile = sys.argv[1]
    
    # Load bag
    print "Opening bag file <%s>" % bagfile
    bag = rosbag.Bag(bagfile)
    
    hz = 100
    
    # Get dictionary of topics
    # topics = list(set([(c.topic, c.datatype) for c in bag._get_connections() if c.topic.startswith("/cf")]))
    topics = list(set([c.topic for c in bag._get_connections() if c.topic.startswith("/cf") or c.topic=='/pid']))
    
    print "Analysing"
    
    master = {"Time":{"":0}}       # (topic, field) -> id
    
    i=1
    stamp_start = None # first measurement
    stamp_end   = 0.0 # last measurement
    printed = False
    for topic, msg, t in bag.read_messages(topics=topics):
        # Get min/max timestamp within selected topics
        ti = t.to_sec()
        if stamp_start == None:
            stamp_start = ti
        else:
            stamp_start = min(stamp_start, ti )
        stamp_end = max(stamp_end, ti)
        
        # Build ID dictionary (topic, field) -> id and reverse
        (fieldnames, types) = msg.__slots__, msg._get_types()    
        if not master.has_key(topic):
            #slots = {name:id+i for (id,name) in enumerate(fieldnames) if name!=("header")}            
            slots = {}
            for name in fieldnames:    
                contents = msg.__getattribute__(name)
                datatype = type(contents)#types[id]                
                if name!=("header"):                                
                    if datatype == tuple:
                        for x in range(len(contents)):
                            slots[name+"_"+str(x)]=i
                            i += 1
                    else:                
                        # Not a tuple
                        slots[name]=i
                        i += 1
            

            master[topic] = slots        
            # break early condition?
        
        td = round(stamp_end-stamp_start)     
        
        if printed:
            if td%15!=0:
                printed = False
                
        if td%15==0 and not printed:
            print td,"s"
            printed = True      
            
      
    master_rev = {}   # id -> (topic, field)
    ids = []
    for topic, dic in master.items():
        for field, id in dic.items():
            master_rev[id] = (topic, field)   
            ids.append(id)
    
    # Calculate duration
    duration = stamp_end - stamp_start
    
    # Given hz and duration, calculate how many timesteps we need to cover whole timespan with equal timesteps
    rows = int(np.ceil(hz*duration))
    
    # Allocate data
    data = np.zeros((rows,len(master_rev)))
    
    # Fill in time in first column
    data[:,0] = np.arange(0,duration,1./hz)
    
    data_new = {topic:np.zeros((rows,1),dtype=np.bool) for topic in topics}  
    
    
    
    
    # Create and init itemgetter for messages 
    msgs = bag.read_messages(topics=topics)   
    topic, msg, stamp = msgs.next() 
    
    print "Gathering"
    # Go through each line of data
    for idx in range(data.shape[0]):    
            
        # Use update current row using previous row
        data[idx,1:] = data[idx-1,1:]       
            
        # Get Data Time      
        tData = data[idx,0]
                   
        # keep adding messages until time too current
        while True:
            tMsg  = stamp.to_sec()-stamp_start 
            if tMsg >= tData:
                break
            for field in [name for name in msg.__slots__ if name!=("header")]:  
                contents = msg.__getattribute__(field)
                datatype = type(contents)      
                if datatype == tuple:
                    for x in range(len(contents)):                            
                        data[idx,master[topic][ field+"_"+str(x) ]] = contents[x]
                        #data_new[idx,master[topic][ field+"_"+str(x) ]] = True                    
                else:
                    data[idx,master[topic][field]] = contents
                    #data_new[idx,master[topic][field]] = True
                    
            data_new[topic][idx] = True
            topic, msg, stamp = msgs.next() 
            
            
        if idx%(data.shape[0]/10) == 0:
            print round(idx*100./data.shape[0],2),"%"
            
    print "Saving as mat" 
    outmat = {"Time":data[:,0]}
    for group, fieldid in master.items():
        if group == "Time":
            print group
            continue
        group_str = group.replace("/cf0", "", 1)
        group_str = group_str.replace("/", "_")
        if group_str[0]=="_":
            group_str = group_str[1:]     
        if group_str[-1]=="_":
            group_str = group_str[:-1]          
        outmat[group_str]={}
        #outmat[group_str+"_new"]={}        
        print group_str
        
        outmat[group_str]["new"] = data_new[group]        
        for field, id in fieldid.items():
            print "\t",field
            outmat[group_str][field] = data[:,id]
            #outmat[group_str+"_new"][field] = data_new[:,id] 
    
    
    print "Saved:", outmat.keys()                
    sio.savemat(bagfile[:-4]+'.mat', {"CFD_"+bagfile[:-4].replace('-','_'):outmat},oned_as='column')
           
if __name__ == "__main__":
    main()
   
