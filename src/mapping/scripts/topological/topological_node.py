import dijkstra as dj
from graph import Graph
from cic_map import getMap

if __name__ == '__main__':

    g = getMap()

    print 'Graph data:'
    for v in g:
        for w in v.get_connections():
            vid = v.get_id()
            wid = w.get_id()
            print '( %s , %s, %3d)'  % ( vid, wid, v.get_weight(w))

    dj.dijkstra(g, g.get_vertex('1'), g.get_vertex('18'))

    target = g.get_vertex('18')
    path = [target]
    dj.shortest(target, path)
    print 'The shortest path : '
    for node in path:
        print 'Node [', node.get_id(), '] (', node.get_pointX(), ', ', node.get_pointY(), ')'
