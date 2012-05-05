Format of the 2D graph files:

Every line in the file specifies either one vertex or one edge

The vertices are specified as follws:
VERTEX2 id x y orientation 
(A 2D node in the graph)

EDGE2 observed_vertex_id observing_vertex_id forward sideward rotate inf_ff inf_fs inf_ss inf_rr inf_fr inf_sr 
(A 2D-edge in the graph. inf_xx are the information matrix entries of the constraint)

EQUIV id1 id2 
(Equivalence constraints between nodes. It merges the node id1 and id2 wrt to the constraint between both vertices.)
