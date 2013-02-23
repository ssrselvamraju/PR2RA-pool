#!/usr/bin/env python

import roslib; roslib.load_manifest("billiards_table")

import random
import billiards_msgs.msg
import math

random.seed()


def generate_hypothesis(x_mean, y_mean, th_mean, x_sdev, y_sdev, th_sdev):
    """
    Generate a gausian hypothesis
    """
    return(random.normalvariate(x_mean,  x_sdev), 
           random.normalvariate(y_mean,  y_sdev), 
           random.normalvariate(th_mean, th_sdev))
    

def generate_hypotheses(x_mean, y_mean, th_mean, x_sdev, y_sdev, th_sdev, number):
    """
    Generate n hypotheses
    """
    out = []
    for i in xrange(0, number):
        out.append(generate_hypothesis(x_mean, y_mean, th_mean, x_sdev, y_sdev, th_sdev))
    return out



def diamond_positions():
    """
    Get the position of the diamonds in x y
    """
    length = billiards_msgs.msg.Constants.TABLE_LENGTH
    width = billiards_msgs.msg.Constants.TABLE_WIDTH
    setback = billiards_msgs.msg.Constants.BUMPER_TO_DIAMOND

    #sequence of fractional positions
    side_sequence = [ 1, 2, 3, 5, 6, 7 ]
    front_sequence = [ 1, 3]
    end_sequence = [ 1, 2, 3]
    

    positions = []
    #sides
    for p in side_sequence:
        #right side
        positions.append((length/8.0*p, -setback))
        # left side
        positions.append((length/8.0*p, width + setback))

    #ends
    for p in front_sequence:
        #near end
        positions.append((-setback, width/4.0*p))

    for p in end_sequence:
        # far end
        positions.append((length + setback, width/4.0*p))

    return positions




def computed_hypothesized_diamonds(offset):
    """
    Return the diamond subject to (x, y th) offset

    """
    x = offset[0]
    y = offset[1]
    th = offset[2]

    output = []
    for d in diamond_positions():
        dx = d[0]
        dy = d[1]
        output.append(( dx * math.cos(th) +dy * math.sin(th) + x, -dx * math.sin(th) + dy * math.cos(th)+ y))

    return output


def point_distance2(p1, p2):
    """
    return the square of the distance between two 2d points
    """
    return (p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1])

def min_point_distance2(point, points):
    """
    FInd the min distance between a point and points
    """
    min_distance = 0.3**2 #hack max value
    for p in points:
        min_distance = min(min_distance, point_distance2(point, p))
    return min_distance


def computed_error(hypothesis, points):
    """ 
    Compute the error between the hypothesis and a set of points
    """
    offset_diamond_points = computed_hypothesized_diamonds(hypothesis)
    
    errors = []
    for p in points:
        errors.append( min_point_distance2(p, offset_diamond_points))

    #errors.sort()

    net_error = 1
    for e in errors:
        net_error *= 1/(1 + math.exp(-40 * e))
    return net_error

   

def find_best_hypothesis(hypotheses, points):
    """ 
    Return the hypothesis with the minimum error, and the error
    
    common usage (best_hyp, error) = table_score.find_best_hypothesis(generate_hypothesis(x_std_dev, 
                                                                                          y_std_dev, 
                                                                                          th_std_dev),
                                                                      points)
    """
    best_hypothesis = hypotheses[0]
    best_error = computed_error(best_hypothesis, points)
    for h in hypotheses:
        error = computed_error(h, points)
        if error < best_error:
            best_hypothesis = h
            best_error = error

    return (best_hypothesis, best_error)
