function [pylod] = payload_weight (box,extra_load)

pylod =box.n*box.weight+extra_load;


