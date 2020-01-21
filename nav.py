import arcpy
import heapq
import math
import random
import time
arcpy.env.workspace = "D:\pagpag"
roads = "L4_1_BDOT10k__OT_SKDR_L.shp"


class Edge:
    def __init__(self, id, node_from, node_to, length, road_class, dir, korek):
        self.id = id
        self.node_from = node_from
        self.node_to = node_to
        self.length = length
        self.road_class = road_class
        self.direction = dir
        self.opoznienie = korek

    def get_end(self, node):
        return self.node_from if node == self.node_to.id else self.node_to

    def cost_length(self):
        return self.length

    def cost_time(self):
        return (self.length/speed[self.road_class]) + self.opoznienie

speed = {"A": 140 * 1000 / 60,
        "S": 120 * 1000 / 60,
        "GP": 100 * 1000 / 60,
        "G": 60 * 1000 / 60,
        "Z": 50 * 1000 / 60,
        "L": 40 * 1000 / 60,
        "l": 20 * 1000 / 60,
        "D": 30 * 1000 / 60,
        "I": 20 * 1000 / 60}


class Node:
    def __init__(self, id, x, y):
        self.id = str(x) + "," + str(y)
        self.x = x
        self.y = y
        self.edges = [id]
        self.gscore = 0

    def get_neighbours(self):
        return self.edges


def heuristic(a, a2, b, b2):
    return math.sqrt((b - a) ** 2 + (b2 - a2) ** 2)


def reconstruct_path(edge_from, came_from, current):
    path = []
    path_edg = []
    path_length = 0
    while current in came_from:
        path.append(current)
        path_edg.append(edge_from[current])
        path_length = path_length + edges[edge_from[current]].cost_length()
        current = came_from[current]
    return path_edg #path_length


def a(edg, start, end, option):
    closed_set = set()
    start.gscore = 0
    came_from = {}
    edge_from = {}
    heap = []
    heap.append((0, start))
    while heap:
        iks = heapq.heappop(heap)[1]
        if iks.id == end.id:
            return reconstruct_path(edge_from, came_from, end)
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
                    fscore = tentative_g_score + heuristic(igrek.x, igrek.y, end.x, end.y)
                elif option == 'czas':
                    fscore = tentative_g_score + heuristic(igrek.x, igrek.y, end.x, end.y) / (120 * 1000 / 60)
                heapq.heappush(heap, (fscore, igrek))
    return False


def dijkstry(edg, start, end):
    closed_set = set()
    start.gscore = 0
    came_from = {}
    edge_from = {}
    heap = []
    heap.append((0, start))
    while heap:
        iks = heapq.heappop(heap)[1]
        if iks.id == end.id:
            return reconstruct_path(edge_from, came_from, end)
        closed_set.add(iks)
        neighbours = iks.get_neighbours()
        for next in neighbours:
            igrek = edg[next].get_end((str(iks.x) + "," + str(iks.y)))
            tentative_g_score = iks.gscore + edges[next].cost_length()
            if igrek in closed_set:
                continue
            if tentative_g_score < igrek.gscore or igrek not in [i[1]for i in heap]:
                came_from[igrek] = iks
                edge_from[igrek] = next
                igrek.gscore = tentative_g_score
                heapq.heappush(heap, (tentative_g_score, igrek))
    return False

edges = {}
nodes = {}

with arcpy.da.SearchCursor(roads, ['ID1', 'klasaDrogi', "SHAPE@"]) as cursor:
    for row in cursor:
        x_start = int(row[2].firstPoint.X)
        y_start = int(row[2].firstPoint.Y)
        x_last = int(row[2].lastPoint.X)
        y_last = int(row[2].lastPoint.Y)
        length = row[2].length
        id = row[0]
        road_class = row[1]

        # generowanie kierunkowosci
        dire = random.randrange(40)
        if dire > 3:
            dire = 0

        if (str(x_start) + "," + str(y_start)) in nodes:
            if dire == 0 or dire == 1:
                nodes[(str(x_start) + "," + str(y_start))].edges.append(id)
        else:
            nodes[str(x_start) + "," + str(y_start)] = Node(id, x_start, y_start)
            if dire == 2 or dire == 3:
                nodes[str(x_start) + "," + str(y_start)].edges.remove(id)

        if (str(x_last) + "," + str(y_last)) in nodes:
            if dire == 0 or dire == 2:
                nodes[(str(x_last) + "," + str(y_last))].edges.append(id)
        else:
            nodes[str(x_last) + "," + str(y_last)] = Node(id, x_last, y_last)
            if dire == 1 or dire == 3:
                nodes[str(x_last) + "," + str(y_last)].edges.remove(id)

        edges[id] = Edge(id, nodes[(str(x_start) + "," + str(y_start))], nodes[(str(x_last) + "," + str(y_last))], length, road_class, dire, 0)

print("wczytane")

# start = edges[9471].node_to
# end = edges[5967].node_from
# (470065.447, 567948.902)
start_point = arcpy.Point(float(raw_input('X start:')), float(raw_input('Y start:')))
# (472828.33, 575517.228)
end_point = arcpy.Point(float(raw_input('X koniec:')), float(raw_input('Y koniec:')))

arcpy.CreateFeatureclass_management("D:\pagpag", "start_stop.shp", "POINT", spatial_reference = roads)
cursor = arcpy.da.InsertCursor('D:\pagpag\start_stop.shp', ['SHAPE@'])
cursor.insertRow([start_point])
cursor.insertRow([end_point])
del cursor

arcpy.Near_analysis('start_stop.shp', roads)

closest = []
cursor = arcpy.da.SearchCursor("start_stop.shp", ["NEAR_FID"])
for row in cursor:
    closest.append(row[0])
del cursor

to_start = heuristic(start_point.X, start_point.Y, edges[closest[0] + 1].node_to.x, edges[closest[0] + 1].node_to.y)
from_start = heuristic(start_point.X, start_point.Y, edges[closest[0] + 1].node_from.x, edges[closest[0] + 1].node_from.y)
to_end = heuristic(end_point.X, end_point.Y, edges[closest[1] + 1].node_to.x, edges[closest[1] + 1].node_to.y)
from_end = heuristic(end_point.X, end_point.Y, edges[closest[1] + 1].node_from.x, edges[closest[1] + 1].node_from.y)

start = edges[closest[0] + 1].node_to if to_start < from_start else edges[closest[0] + 1].node_from
end = edges[closest[1] + 1].node_to if to_end < from_end else edges[closest[1] + 1].node_from

opcja = raw_input('Trasa wedlug dlugosc/czas:')

#start_time = time.time()
b = (a(edges, start, end, opcja))
#b = dijkstry(edges,start,end)
#print("--- Algorytm wykonano w %s sekund ---" % (time.time() - start_time))
czy_korek = raw_input('Jest korek(tak/nie)?')

if b and czy_korek == 'tak':
    korek_dlugosc = float(raw_input('Jak dlugi jest korek?'))
    korek_czas = float(raw_input('Ile traci sie czasu(minuty)?'))
    korek_poczatek = korek_dlugosc
    for i in range (1,1000):
        if float(edges[b[-i]].length) <= korek_dlugosc:
            korek_dlugosc = korek_dlugosc - float(edges[b[-i]].length)
            czesc_korka = float(edges[b[-i]].length) / korek_poczatek
            edges[b[-i]].opoznienie = int(korek_czas * czesc_korka)
        else:
            edges[b[-i]].opoznienie = int(korek_czas * (1 - czesc_korka))
            b = (a(edges, start, end, opcja))
            break
array = arcpy.Array()
second = arcpy.Point()

if b:
    qry = """ID1 IN {0}""".format(str(tuple(b)))
    arcpy.Select_analysis(roads, "D:/pagpag/trasa2.shp", qry)
    array.add(start_point)
    second.X = start.x
    second.Y = start.y
    array.add(second)
    polyline = arcpy.Polyline(array)
    arcpy.CreateFeatureclass_management("D:\pagpag", "start_stop_line.shp", "POLYLINE", spatial_reference=roads)
    cursor = arcpy.da.InsertCursor('D:\pagpag\start_stop_line.shp', ['SHAPE@'])
    cursor.insertRow([polyline])
    array.removeAll()
    second.X = end.x
    second.Y = end.y
    array.add(second)
    array.add(end_point)
    polyline_end = arcpy.Polyline(array)
    cursor.insertRow([polyline_end])
    del cursor
else:
    print("Droga nieznaleziona")