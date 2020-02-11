#!/usr/bin/env python
# coding: utf-8

# In[17]:


from leuvenmapmatching.map.inmem import InMemMap
from leuvenmapmatching.matcher.distance import DistanceMatcher
import leuvenmapmatching.visualization as mm_viz
import geopandas as gpd
import pandas as pd
import smopy
from rdp import rdp
#import folium
#from matplotlib import pyplot as plt

import osmnx as ox
import csv


# # Open street map 가져오기

# In[3]:


def get_graph(base, dis):
    graph = ox.graph_from_point(base, distance = dis, network_type='drive')
    graph_proj = ox.project_graph(graph)
    
    return graph_proj


# In[4]:


'''
base = (37.492797, 127.046334)
length = 8000
G = get_graph(base, length)
'''


# In[3]:


'''
base = (37.492797, 127.046334)
graph = ox.graph_from_point(base, distance = 8000, network_type='drive')
graph_proj = ox.project_graph(graph)

#ox.plot_graph(graph, fig_height = 10, fig_width = 10, edge_color='black')
'''


# In[6]:


# leuven map matching 적용위한 맵 생성

def mk_map(graph_proj):
    map_con = InMemMap("myosm", use_latlon=True, use_rtree=True, index_edges=True)

    #Create GeoDataFrames
    nodes_proj, edges_proj = ox.graph_to_gdfs(graph_proj, nodes=True, edges=True)

    #좌표계 변경
    nodes_proj = nodes_proj.to_crs({'init': 'epsg:4326'})
    edges_proj = edges_proj.to_crs({'init': 'epsg:4326'})

    for nid, row in nodes_proj[['lon', 'lat']].iterrows():
        map_con.add_node(nid, (row['lat'], row['lon']))
    for nid, row in edges_proj[['u', 'v']].iterrows():
        map_con.add_edge(row['u'],  row['v'])
        
    return map_con


# In[5]:


'''
map_con = InMemMap("myosm", use_latlon=True, use_rtree=True, index_edges=True)

#Create GeoDataFrames
nodes_proj, edges_proj = ox.graph_to_gdfs(graph_proj, nodes=True, edges=True)

#좌표계 변경
nodes_proj = nodes_proj.to_crs({'init': 'epsg:4326'})
edges_proj = edges_proj.to_crs({'init': 'epsg:4326'})

for nid, row in nodes_proj[['lon', 'lat']].iterrows():
    map_con.add_node(nid, (row['lat'], row['lon']))
for nid, row in edges_proj[['u', 'v']].iterrows():
    map_con.add_edge(row['u'],  row['v'])
'''


# # gps 데이터 가져오기

# In[7]:


def get_track(loc):
    #gps data 불러오기
    gps_data = pd.read_csv(loc)

    #gps데이터에서 생성시간, 위도, 경도 데이터만 나타내기
    gps_data = gps_data[['latitude', 'longitude']]

    #이상치 제거 - 위도, 경도가 0이고, 속도가 0인 것(정지상태)
    gps_data = gps_data[gps_data.latitude !=0]
    gps_data = gps_data[gps_data.longitude !=0]
    #gps_data = gps_data[gps_data.speed !="0.0mph"]
    
    #rdp 미적용
    route_list = [tuple(x) for x in gps_data.to_numpy()]
    
    #rdp 적용
    #Florian Wilhelm - Handling GPS Data with Python https://youtu.be/9Q8nEA_0ccg
    #route = rdp(gps_data[['latitude','longitude']].values, epsilon=1e-5)

    #route(array 형식)를 list 형식으로 바꾸기
    #route_list = [tuple(x) for x in route.tolist()]
    
    #일부 데이터만 이용 -> 나중에는 없애기
    track = route_list[:500]
    
    return track


# In[8]:


'''
#gps data 불러오기
gps_data = pd.read_csv('Task1_20191105.csv')

#gps데이터에서 생성시간, 위도, 경도 데이터만 나타내기
gps_data = gps_data[['latitude', 'longitude']]

#이상치 제거 - 위도, 경도가 0이고, 속도가 0인 것(정지상태)
gps_data = gps_data[gps_data.latitude !=0]
gps_data = gps_data[gps_data.longitude !=0]
#gps_data = gps_data[gps_data.speed !="0.0mph"]

#rdp 미적용
route_list = [tuple(x) for x in gps_data.to_numpy()]

#rdp 적용
#Florian Wilhelm - Handling GPS Data with Python https://youtu.be/9Q8nEA_0ccg
#route = rdp(gps_data[['latitude','longitude']].values, epsilon=1e-5)

#route(array 형식)를 list 형식으로 바꾸기
#route_list = [tuple(x) for x in route.tolist()]

#route_list 중 100개만 
track = route_list[:500]
'''


# # Leuven map matching 적용하기

# In[9]:


def dis_matcher(map_con, track):
    matcher = DistanceMatcher(map_con,
                              max_dist = 200, min_prob_norm=0.0001,
                              non_emitting_length_factor=0.5,
                              obs_noise=10, obs_noise_ne=10,
                              dist_noise=100,
                              max_lattice_width=5, avoid_goingback=True,
                              non_emitting_states=True)
    states, last_idx = matcher.match(track)

    nodes = matcher.path_pred_onlynodes
    
    return matcher, nodes


# In[9]:


'''
matcher = DistanceMatcher(map_con,
                              max_dist = 200, min_prob_norm=0.0001,
                              non_emitting_length_factor=0.5,
                              obs_noise=10, obs_noise_ne=10,
                              dist_noise=100,
                              max_lattice_width=5, avoid_goingback=True,
                              non_emitting_states=True)
states, last_idx = matcher.match(track)

nodes = matcher.path_pred_onlynodes
'''


# In[10]:


'''
#leuven map matching내의 메소드 이용하여 시각화
mm_viz.plot_map(map_con, matcher=matcher, use_osm=True,show_labels=False,
                zoom_path=True, show_graph=False, show_matching=True,
                filename=str("result.png"))

'''


# In[10]:


def to_pixels(lat, lon=None):
    if lon is None:
        lat, lon = lat[0], lat[1]
    return lon, lat


# In[11]:


#matching 노드의 좌표값 찾기
def find_mid_nodes(map_con, matcher):
    coords_list = []
    coord_trans = None
    ax = None
    
    z=18
    
    bb = map_con.bb()    
    m = smopy.Map(bb, z=z, ax=ax)
    
    if matcher is not None:
        lat_nodes = matcher.lattice_best
    else:
        lat_nodes = None
    
    for idx, m in enumerate(lat_nodes):
        lat, lon = m.edge_m.pi[:2]
        #lat2, lon2 = m.edge_o.pi[:2]
        
        if coord_trans:
         #   lat, lon = coord_trans(lat, lon)
            lat2, lon2 = coord_trans(lat2, lon2)
    
        x,y = to_pixels(lat,lon)
        #x2, y2 = to_pixels(lat2, lon2)
        coords_list.append((y,x))   

    return coords_list


# In[12]:


def get_match_coords(G, nodes):
    match_coords_list = match_nodes_geom(graph, nodes)
    
    return match_coords_list


# # Folium 이용한 시각화
# 
# 중간에 매칭된 mid node 좌표 찾기 (mm_viz로 그린 결과에서 초록색 x)
# 다 안해도 됨

# In[14]:


'''
#base_map 형식 지정하기
def generateBaseMap(default_location = [37.492397, 127.039084], default_zoom_start = 12):
    base_map = folium.Map(location = default_location, control_scale = True, zoom_start = default_zoom_start)
    return base_map

def match_nodes_geom(G, nodes_list):
    geom = []
    ex_n = len(nodes)
    
    for i in range(0, ex_n):
        t = (G.nodes[nodes_list[i]]['y'], G.nodes[nodes_list[i]]['x'])
        geom.append(t)
        
    return geom

def mk_match_folium(match_coords_list):
    match_base_map = generateBaseMap()

    #gps 데이터를 나타내기
    for each in match_coords_list:
        folium.Marker(each).add_to(match_base_map)


    folium.PolyLine(match_coords_list).add_to(match_base_map)

    match_base_map.save("match_nodes.html")
    
def mk_mid_folium(mid_coords_list):
    mid_base_map = generateBaseMap()

    for each in mid_coords_list:
        folium.Marker(each).add_to(mid_base_map)

    folium.PolyLine(mid_coords_list).add_to(mid_base_map)

    mid_base_map.save("mid_nodes.html")
'''


# In[15]:


'''
match_coords_list = match_nodes_geom(graph, nodes)

mid_coords_list = []
find_mid_nodes(matcher, mid_coords_list)
'''


# In[16]:


'''
match_base_map = generateBaseMap()

#gps 데이터를 나타내기
for each in match_coords_list:
    folium.Marker(each).add_to(match_base_map)


folium.PolyLine(match_coords_list).add_to(match_base_map)

match_base_map.save("match_nodes.html")

mid_coords_list = []
find_mid_nodes(matcher, mid_coords_list)

mid_base_map = generateBaseMap()

for each in mid_coords_list:
    folium.Marker(each).add_to(mid_base_map)
    
folium.PolyLine(mid_coords_list).add_to(mid_base_map)

mid_base_map.save("mid_nodes.html")
'''


# In[15]:


def main():
  
    base = (37.492797, 127.046334)
    length = 8000
    G = get_graph(base, length)   
    
    loc = 'Task1_20191105.csv'
    track = get_track(loc)
    
    map = mk_map(G)
    
    matcher, nodes = dis_matcher(map, track)
    
    coords_list = find_mid_nodes(map, matcher)
    
    return coords_list


# In[19]:


if __name__ == '__main__':
    coords = main()
    
    print(coords)
    
    #route 리스트를 csv 파일로 저장
    csvfile = open("coords.csv", "w", newline="")
    
    csvwriter = csv.writer(csvfile)
    for row in coords:
        csvwriter.writerow(row)
        
    csvfile.close()

