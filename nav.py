import arcpy
import heapq
import math
arcpy.env.workspace = "D:\pagpag"
roads = "L4_1_BDOT10k__OT_SKDR_L.shp"


class Edge:
    def __init__(self, id, node_from, node_to, length, road_class):
        self.id = id
        self.node_from = node_from
        self.node_to = node_to
        self.length = length
        self.road_class = road_class

    def get_end(self,node):
        return (self.node_from if node == self.node_to.id else self.node_to)

    def cost_length(self):
        return self.length

    def cost_time(self):
        return float(self.length/self.speed[self.road_class])

    speed = {"A":140,
             "S":120,
             "GP":100,
             "G":60,
             "Z":50,
             "L":40,
             "D":30,
             "I":20}


class Node:
    def __init__(self, id, x, y):
        self.id = str(x) + "," + str(y)
        self.x = x
        self.y = y
        self.edges = [(id)]
        self.gscore = 0

    def get_neighbours(self):
        return self.edges


def heuristic(a,a2, b, b2):
    return math.sqrt((b - a) ** 2 + (b2 - a2) ** 2)



def reconstruct_path(edge_from,came_from, current):
    path = []
    path_edg = []
    len = 0
    while current in came_from:
        path.append(current)
        path_edg.append(edge_from[current])
        len = len + edges[edge_from[current]].cost_length()
        current = came_from[current]
    return path_edg



def a(edg,start,end,option):
    closed_set = set()
    start.gscore = 0
    came_from = {}
    edge_from = {}
    heap = []
    heap.append((0,start))
    while heap:
        iks = heapq.heappop(heap)[1]
        if iks.id == end.id:
            return reconstruct_path(edge_from,came_from, end)
        closed_set.add(iks)
        neighbours = iks.get_neighbours()
        for next in neighbours:
            igrek = edg[next].get_end((str(iks.x) + "," + str(iks.y)))
            if option == 'dlugosc':
                tentative_g_score = iks.gscore + edges[next].cost_length()
            elif option == 'czas':
                tentative_g_score = iks.gscore + edges[next].cost_time()
            if igrek in closed_set:
                continue
            if tentative_g_score < igrek.gscore or igrek not in [i[1]for i in heap]:
                came_from[igrek] = iks
                edge_from[igrek] = next
                igrek.gscore = tentative_g_score
                if option == 'dlugosc':
                    fscore = tentative_g_score + heuristic(igrek.x,igrek.y,end.x,end.y)
                elif option == 'czas':
                    fscore = tentative_g_score + heuristic(igrek.x,igrek.y,end.x,end.y)/70
                heapq.heappush(heap, (fscore, igrek))
    return false

edges = {}
nodes = {}

with arcpy.da.SearchCursor(roads, ['ID1','klasaDrogi',"SHAPE@"]) as cursor:
    for row in cursor:
        x_start = row[2].firstPoint.X
        y_start = row[2].firstPoint.Y
        x_last = row[2].lastPoint.X
        y_last = row[2].lastPoint.Y
        length = row[2].length
        id = row[0]
        road_class = row[1]


        if (str(x_start) + "," + str(y_start)) in nodes:
            nodes[(str(x_start) + "," + str(y_start))].edges.append(id)
        else:
            nodes[str(x_start) + "," + str(y_start)] = Node(id, x_start,y_start)

        if (str(x_last) + "," + str(y_last)) in nodes:
            nodes[(str(x_last) + "," + str(y_last))].edges.append(id)
        else:
            nodes[str(x_last) + "," + str(y_last)] = Node(id, x_last,y_last)


        edges[id] = Edge(id,nodes[(str(x_start) + "," + str(y_start))], nodes[(str(x_last) + "," + str(y_last))],length,road_class )

print("wczytane")

start = edges[9471].node_to
end = edges[5967].node_from


opcja = raw_input('Trasa wedlug dlugosc/czas:')

b = (a(edges,start,end,opcja))

qry = """ID1 IN {0}""".format(str(tuple(b)))

arcpy.Select_analysis(roads, "D:/pagpag/trasa.shp",qry)