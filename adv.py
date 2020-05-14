from room import Room
from player import Player
from world import World

from collections import defaultdict

from util import Stack, Queue  # These may come in handy

import random
from ast import literal_eval

# int mapping to a cardinal direction so we can get the next direction using ith_neighbor + 1
cardinal_directions = [ 'n', 's', 'e', 'w' ]

opposite_directions = {
    'n': 's',
    's': 'n',
    'e': 'w',
    'w': 'e'
}

def dead_end(room, player_path_log):

    # player_path_log is a list of tuples, where each tuple holds the direction to the current room
    # and the current room id


    # we can only move backward if a dead end is reached

    room_directions = {
        'n': room.n_to,
        's': room.s_to,
        'e': room.e_to,
        'w': room.w_to
    }

    # the last entry in the log tells us the direction and the room 
    remaining_directions = {i: room_directions[i] for i in room_directions if room_directions[i] is not None}

    # only 1 direction possible
    if len(remaining_directions.keys()) == 1:
        # print(player_path_log[-1])
        # print(opposite_directions[player_path_log[-1][0]])
        # print(remaining_directions)
        # player can only go backwards from where they last came from
        if remaining_directions[  opposite_directions[player_path_log[-1][0]] ].id > -1:
            return True
    return False

# this is an unvisited room function
def is_unvisited(room):

    directions = []
    directions.append(room.n_to)
    directions.append(room.s_to)
    directions.append(room.e_to)
    directions.append(room.w_to)

    return len([i for i in directions if i != '?']) == 0

def find_unvisited_rooms(room, room_id, world):

    # print(room)
    # this room is on the player's map

    # if all rooms have been explored or None then this returns []

    directions = []
    if room['n'] == '?':
        north_neighbor = world.rooms[ room_id ].n_to
        if north_neighbor is not None:
            directions.append(('n', north_neighbor))

    if room['s'] == '?':
        south_neighbor = world.rooms[ room_id ].s_to
        if south_neighbor is not None:
            directions.append(('s', south_neighbor))

    if room['e'] == '?':
        east_neighbor = world.rooms[ room_id ].e_to
        if east_neighbor is not None:
            directions.append(('e', east_neighbor))

    if room['w'] == '?':
        west_neighbor = world.rooms[ room_id ].w_to
        if west_neighbor is not None:
            directions.append(('w', west_neighbor))
    
    return directions

def bfs(world, player, player_path_log):
    """
    Return a list containing the shortest path from
    starting_vertex to destination_vertex in
    breath-first order.
    """
    pass  # TODO
    # do bfs till we find a node with at least 1 unvisited edge
    # [print(i, self.vertices[i]) for i in self.vertices]
    visited = {i: 0 for i in self.vertices}

    my_queue = Queue()
    my_queue.enqueue(starting_vertex)
    current_node = -1

    # this is how we remember the path from the start vertex to the
    # destination(reversed linked list)
    parents = {i: 0 for i in self.vertices}

    while current_node != destination_vertex:

        current_node = my_queue.dequeue()

        if visited[current_node] == 0:
            # print(current_node)

            visited[current_node] = 1
            for node in self.vertices[current_node]:
                my_queue.enqueue(node)
                parents[node] = current_node
    # [print(i, parents[i]) for i in parents]
    tracker = destination_vertex
    search_path = []
    while tracker > 0:
        search_path = [tracker, *search_path]
        tracker = parents[tracker]
    # print(search_path)
    return search_path

def dft(world, player):
    """
    Print each vertex in depth-first order
    beginning from starting_vertex.
    """
    pass  # TODO
    # rooms is the graph, ordered by cardinal direction
    # the starting vertex is the room the player is on
    # as we push and pop from stack the player travels forward and backwards through the rooms
    # this means the reverse directions must be set when the player goes forward
    # print('here')
    # graph_for_stack = {i: list(self.vertices[i]) for i in self.vertices}
    # my_stack = Stack()
    # current = {'current': starting_vertex, 'ith_neighbor': 0}
    # [print(i, rooms[i]) for i in rooms]
    # change direction when we hit a dead end or node has all paths 
    visited = {i: 0 for i in world.rooms}
    my_map = {0: {'n': '?', 's': '?', 'w': '?', 'e': '?'}}
    count = 0
    player.current_room = world.rooms[0]
    # visited[player.current_room.id] = 1
    player_path_log = [('start', 0)]
    # print(not(my_stack.size() == 0) or current['current'] in graph_for_stack)
    # (current is not none) and (we have a stck or node is in graph)
    # stop when we have visited all nodes or when count is large enough
    # while (current is not None) and (not(my_stack.size() == 0) or current['current'] in graph_for_stack):
    while len([visited[room] for room in visited if visited[room] == 0]) > 0:

        if count == 3:
            print('loop has reached it\'s limit')
            return
        print('current count', count)
        print('player\'s current room', player.current_room.id)
        # print(my_stack.stack)
        # print(my_stack.stack, current['current'], graph_for_stack[ current['current'] ], visited[ current['current'] ])
        # def travel(self, direction, show_rooms = False):

        # the current node is in the graph and it hasn't been visited
        # if current['current'] in graph_for_stack and visited[ current['current'] ] == 0:
        # print(dead_end( world.rooms[ player.current_room.id ], player_path_log))
        # print(visited[ player.current_room.id ])
        if not dead_end( world.rooms[ player.current_room.id ], player_path_log) and visited[ player.current_room.id ] == 0:

            # We know there is at least 1 room unvisited
            visited[ player.current_room.id ] = 1

            # print(player.current_room)

            # this is why each set of vertices needs to be an array
            # my_stack.push({'current': current['current'], 'ith_neighbor': 0})
            # current = { 'current': graph_for_stack[ current['current'] ][0],
            #             'ith_neighbor': 0}
            # should only choose unvisited rooms that exist
            unvisited_rooms = find_unvisited_rooms( my_map[player.current_room.id],
                                                    player.current_room.id,
                                                    world)

            if len(unvisited_rooms) > 0:
                # assume these unvisited rooms exist 
                # print('rooms to visit')
                # [print(i) for i in unvisited_rooms]

                # [print(i[0], i[1].id) for i in unvisited_rooms if i[1] is not None]
                # get the first one
                next_room_to_visit = unvisited_rooms[0]
                # print('my next room to see')
                # print(next_room_to_visit[0], next_room_to_visit[1].id)
                next_direction = next_room_to_visit[0]
                next_room_id = next_room_to_visit[1].id
                # print('next room to visit')
                # print(next_direction, next_room_id)
                # add next room to map
                my_map[next_room_id] =  {'n': '?', 's': '?', 'w': '?', 'e': '?'}

                # link current room to next room
                my_map[player.current_room.id][next_direction] = next_room_id
                # link next room to current room
                my_map[next_room_id][ opposite_directions[next_direction] ] = player.current_room.id

                # move player to next room
                player.travel(next_direction)
                player_path_log.append((next_direction, next_room_id))
                print('player\'s room', player.current_room.id)

                [print(i, my_map[i]) for i in my_map]
                [print(i) for i in player_path_log]
                print()
                    # exit()
            else:
                # we cannot move forward any more
                # the player cannot go forward in any other direction so set all the rest of the unexplored paths to None
                for i in my_map[ player.current_room.id ]:
                    if my_map[ player.current_room.id ][i] == '?':
                        my_map[ player.current_room.id ][i] = None
                # bfs in the opposite direction
                print('bfs in the opposite direction')
        else:
            # we know the player is at a dead end so set all the rest of the unexplored paths to None
            for i in my_map[ player.current_room.id ]:
                if my_map[ player.current_room.id ][i] == '?':
                    my_map[ player.current_room.id ][i] = None
            
            # need to find the closest node with at least 1 room unexplored to restart dft

            # print('here')
            print(player.current_room)

            [print(i, my_map[i]) for i in my_map]
            # do bfs in the opposite direction
            # exit()
            # # bfs to the nearest node with an unvisited edge
            # # node doesn't exist or we are revisiting a node
            # current = my_stack.pop()
            # if current is not None:
            #     current['ith_neighbor'] += 1
            
            #     # set current to the current's current at the current's ith neighbor
            #     if current['ith_neighbor'] < len( graph_for_stack[ current['current'] ] ):
            #         current['current'] = graph_for_stack[ current['current'] ][ current['ith_neighbor'] ]
        count += 1

# Load world
world = World()


# You may uncomment the smaller graphs for development and testing purposes.
# map_file = "maps/test_line.txt"
# map_file = "maps/test_cross.txt"
map_file = "maps/test_loop.txt"
# map_file = "maps/test_loop_fork.txt"
# map_file = "maps/main_maze.txt"

# Loads the map into a dictionary
room_graph=literal_eval(open(map_file, "r").read())
world.load_graph(room_graph)

# Print an ASCII map
world.print_rooms()

player = Player(world.starting_room)

# Fill this out with directions to walk
# traversal_path = ['n', 'n']
traversal_path = []

print('player')
print(player)
print('done')
dft(world, player)


exit()
# TRAVERSAL TEST - DO NOT MODIFY
visited_rooms = set()
player.current_room = world.starting_room
visited_rooms.add(player.current_room)

for move in traversal_path:
    player.travel(move)
    visited_rooms.add(player.current_room)

if len(visited_rooms) == len(room_graph):
    print(f"TESTS PASSED: {len(traversal_path)} moves, {len(visited_rooms)} rooms visited")
else:
    print("TESTS FAILED: INCOMPLETE TRAVERSAL")
    print(f"{len(room_graph) - len(visited_rooms)} unvisited rooms")



#######
# UNCOMMENT TO WALK AROUND
#######
player.current_room.print_room_description(player)
while True:
    cmds = input("-> ").lower().split(" ")
    if cmds[0] in ["n", "s", "e", "w"]:
        player.travel(cmds[0], True)
    elif cmds[0] == "q":
        break
    else:
        print("I did not understand that command.")
