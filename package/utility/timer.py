
from datetime import datetime
import numpy as np
import time

class Timer(object):
    """ CONSTRUCTOR """
    def __init__(self,_name='timer',_HZ=10,_MAX_SEC=np.inf,_VERBOSE=True):
        self.name           = _name
        self.HZ             = _HZ
        self.max_sec        = _MAX_SEC
        self.VERBOSE        = _VERBOSE
        self.sec_next       = 0.0
        self.sec_period     = 1.0 / self.HZ
        self.sec_elps       = 0.0
        self.sec_elps_prev  = 0.0
        self.sec_elps_diff  = 0.0
        self.sec_elps_loop  = 0.0 # exact esec 
        self.tick           = 0.
        self.force_finish   = False
        self.DELAYED_FLAG   = False
        if self.VERBOSE & 0:
            print ("[%s] initialized [%d]HZ. MAX_SEC:[%.1fsec]."
                % (self.name, self.HZ, self.max_sec))
    """ START TITMER """
    def start(self):
        self.time_start     = datetime.now()
        self.sec_next       = 0.0
        self.sec_elps       = 0.0
        self.sec_elps_prev  = 0.0
        self.sec_elps_diff  = 0.0
        self.tick           = 0.
        if self.VERBOSE:
            print ("[%s] start ([%d]HZ. MAX_SEC:[%.1fsec])."
                % (self.name, self.HZ, self.max_sec))
    """ FORCE FINISH """
    def finish(self):
        self.force_finish = True
    """ CHECK FINISHED """
    def is_finished(self):
        self.time_diff = datetime.now() - self.time_start
        self.sec_elps  = self.time_diff.total_seconds()
        if self.force_finish:
            return True
        if self.sec_elps > self.max_sec:
            return True
        else:
            return False
    def is_notfinished(self):
        self.time_diff = datetime.now() - self.time_start
        self.sec_elps  = self.time_diff.total_seconds()
        if self.force_finish:
            return False
        if self.sec_elps > self.max_sec:
            return False
        else:
            return True
    """ RUN """
    def do_run(self):
        time.sleep(1e-8) # ADD A SMALL TIME DELAY
        self.time_diff = datetime.now() - self.time_start
        self.sec_elps  = self.time_diff.total_seconds()
        if self.sec_elps > self.sec_next:
            self.sec_next = self.sec_next + self.sec_period
            self.tick     = self.tick + 1
            """ COMPUTE THE TIME DIFFERENCE & UPDATE PREVIOUS ELAPSED TIME """
            self.sec_elps_diff = self.sec_elps - self.sec_elps_prev
            self.sec_elps_prev = self.sec_elps
            """ CHECK DELAYED """
            if (self.sec_elps_diff > self.sec_period*1.5) & (self.HZ != 0):
                if self.VERBOSE:
                    # print ("sec_elps_diff:[%.1fms]" % (self.sec_elps_diff*1000.0))
                    print ("[%s][%d][%.1fs] delayed! T:[%.1fms] But it took [%.1fms]. Acutally [%.1fms]"
                        % (self.name,self.tick, self.sec_elps, self.sec_period*1000.0, 
                        self.sec_elps_diff*1000.0,self.sec_elps_loop*1000.0))
                self.DELAYED_FLAG = True

            # Save when 
            self.time_run_start = datetime.now()
            return True
        else:
            self.DELAYED_FLAG = False
            return False

    # End of loop
    def end(self):
        time_diff = datetime.now() - self.time_run_start
        self.sec_elps_loop = time_diff.total_seconds()
        if (self.sec_elps_loop > self.sec_period) & (self.HZ != 0):
            # Print all the time
            print ("[%s] is REALLY delayed! T:[%.1fms] BUT IT TOOK [%.1fms]"
            % (self.name,self.sec_period*1000.0, self.sec_elps_loop*1000.0))