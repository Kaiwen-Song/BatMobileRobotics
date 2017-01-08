import random
import os
from movementlib import *
from config import *
# Location signature class: stores a signature characterizing one location
class LocationSignature:
    def __init__(self, no_bins = SIG_LENGTH):
        self.sig = [0] * no_bins

    def print_signature(self):
        for i in range(len(self.sig)):
            print self.sig[i]

# --------------------- File management class ---------------
class SignatureContainer():
    def __init__(self, size = 5):
        self.size      = size; # max number of signatures that can be stored
        self.filenames = [];

        # Fills the filenames variable with names like loc_%%.dat
        # where %% are 2 digits (00, 01, 02...) indicating the location number.
        for i in range(self.size):
            self.filenames.append('loc_{0:02d}.dat'.format(i))

    # Get the index of a filename for the new signature. If all filenames are
    # used, it returns -1;
    def get_free_index(self):
        n = 0
        while n < self.size:
            if (os.path.isfile(self.filenames[n]) == False):
                break
            n += 1

        if (n >= self.size):
            return -1;
        else:
            return n;

    # Delete all loc_%%.dat files
    def delete_loc_files(self):
        print "STATUS:  All signature files removed."
        for n in range(self.size):
            if os.path.isfile(self.filenames[n]):
                os.remove(self.filenames[n])

    # Writes the signature to the file identified by index (e.g, if index is 1
    # it will be file loc_01.dat). If file already exists, it will be replaced.
    def save(self, signature, index):
        filename = self.filenames[index]
        if os.path.isfile(filename):
            os.remove(filename)

        f = open(filename, 'w')

        for i in range(len(signature.sig)):
            s = str(signature.sig[i]) + "\n"
            f.write(s)
        f.close();

    # Read signature file identified by index. If the file doesn't exist
    # it returns an empty signature.
    def read(self, index):
        ls = LocationSignature()
        filename = self.filenames[index]
        if os.path.isfile(filename):
            f = open(filename, 'r')
            for i in range(len(ls.sig)):
                s = f.readline()
                if (s != ''):
                    ls.sig[i] = int(s)
            f.close();
        else:
            print "WARNING: Signature does not exist."

        return ls

def trim_signature(ls):
    if len(ls) > SIG_LENGTH:
        overshoot = len(ls) - SIG_LENGTH
        while len(ls) > SIG_LENGTH:
            rand_index = random.randint(0,len(ls))
            del(ls[rand_index])
    return ls

# Need to spin the robot and then fetch the sonar's value. As we are going to
# a reading every 5 degrees, we will have 72 readings in total
def characterize_location(ls, interface):
    print "STATUS: Learning location"
    ls.sig = trim_signature(sonar360Reading(interface))

def rotate_list(l, n):
    return l[n:] + l[:n]

# This function will compare the two signatures and the smaller the value; the
# closer they are
def compare_signatures(ls1, ls2):
    return sum([pow(a - b, 2) for (a, b) in zip(ls1, ls2)])


# This function characterizes the current location, and stores the obtained
# signature into the next available file.
def learn_location(interface):
    ls = LocationSignature()
    characterize_location(ls, interface)
    idx = signatures.get_free_index();
    if (idx == -1): # run out of signature files
        print "\nWARNING:"
        print "No signature file is available. NOTHING NEW will be learned and stored."
        print "Please remove some loc_%%.dat files.\n"
        return

    signatures.save(ls,idx)
    print "STATUS:  Location " + str(idx) + " learned and saved."

# This function tries to recognize the current location.
# 1.   Characterize current location
# 2.   For every learned locations
# 2.1. Read signature of learned location from file
# 2.2. Compare signature to signature coming from actual characterization
# 3.   Retain the learned location whose minimum distance with
#      actual characterization is the smallest.
# 4.   Display the index of the recognized location on the screen

def calculate_buckets(signature):
    buckets = [0 for _ in range(13)]
    for reading in signature:
        buckets[int(reading) / 20] += 1
    return buckets


def recognize_location(interface):
    ls_obs = LocationSignature();
    characterize_location(ls_obs,interface)
    rotateSensor(0.6,interface)
    # ls_obs_sorted = sorted(ls_obs.sig)
    comparison_results = []
    for idx in range(signatures.size):
        print "STATUS:  Comparing signature " + str(idx) + " with the observed signature."
        ls_read = signatures.read(idx).sig
        print "SIGNATURE: " , calculate_buckets(ls_obs.sig)
        print calculate_buckets(ls_read)
        comparison_val = compare_signatures(calculate_buckets(ls_obs.sig), calculate_buckets(ls_read))
        comparison_results.append((idx, comparison_val))
    # Sort the list of tuples and then take the value
    comparison_results = sorted(comparison_results, key=getKey)
    (best_index, comparison_val) = comparison_results[0]
    print comparison_results
    best_sig = signatures.read(best_index)
    min_sum = pow(2,31)
    current_angle = 0
    for i in range(0, SIG_LENGTH):
        result = sum([pow(a - b, 2) for (a, b) in zip(rotate_list(best_sig.sig, i), ls_obs.sig)])
        print result
        if(result < min_sum):
            current_angle = i * 360/SIG_LENGTH
            min_sum = result
    print "BEST RESULT: Location Index " + str(best_index) + " with comparison_val of " + str(comparison_val) + ""
    return LOCATIONS[best_index], current_angle


def getKey(curItem):
    return curItem[1]

# Prior to starting learning the locations, it should delete files from previous
# learning either manually or by calling signatures.delete_loc_files().
# Then, either learn a location, until all the locations are learned, or try to
# recognize one of them, if locations have already been learned.

#
#
signatures = SignatureContainer(5);
print signatures.filenames
# #signatures.delete_loc_files()
#
# learn_location();
# recognize_location();
