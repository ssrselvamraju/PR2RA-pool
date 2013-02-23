#!/usr/bin/env python

import table_score







t = table_score.diamond_positions()

for p in t:
    print "Diamond", p


s = table_score.generate_hypotheses(0,0,0,0.1, 0, 0.01, 3)
for h in s:
    print "hypothesis", h
    th = table_score.computed_hypothesized_diamonds(h)
    for p in th:
        print "twisted diamond", p


print "distance2 between (1,1) and (2,4) is %f"%table_score.point_distance2((1,1), (2,4))
print "min_point_distance2 between (1,1) and [(2,4), (3,4)] is %f"%table_score.min_point_distance2((1,1), [(2,4), (3,4)])



print "errors for points [(0,0), (0,.1)], hypothesis (0,0,0)"
points = [(0,0), (0,.1)]
hypothesis = (0,0,0)

print table_score.computed_error(hypothesis, points)




print "find best hypotheses"
hypotheses = [(0,0,0), (0,0,.01)]
print table_score.find_best_hypothesis(hypotheses, s)



(best_hyp, best_error) = table_score.find_best_hypothesis(table_score.generate_hypotheses(0,0,0,0.1, 0.1, 0.1, 100),
                                                          t)
print "best hyp, best error", best_hyp, best_error
