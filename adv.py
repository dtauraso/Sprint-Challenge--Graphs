from room import Room
from player import Player
from world import World

from collections import defaultdict

from util import Stack, Queue  # These may come in handy

import random
from ast import literal_eval

# int mapping to a cardinal direction so we can get the next direction using ith_neighbor + 1
cardinal_directions = [ 'n', 's', 'e', 'w' ]

# {0: (id, 'n'), 1: (id, 's'), 2: (id, 'e')}
opposite_directions = {
    'n': 's',
    's': 'n',
    'e': 'w',
    'w': 'e'
}

object_directions = {
    'n': 'n_to',
    's': 's_to',
    'e': 'e_to',
    'w': 'w_to'
}
def get_room_directions(room, direction):
    room_directions = {
        'n': room.n_to,
        's': room.s_to,
        'e': room.e_to,
        'w': room.w_to
    }
    return room_directions[direction]

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
        print(player_path_log[-1])
        # special case where the start node is a dead end
        if player_path_log[-1][0] == 'start':
            return False
        # print(opposite_directions[player_path_log[-1][0]])
        # print(remaining_directions)
        # player can only go backwards from where they last came from
        if remaining_directions[  opposite_directions[ player_path_log[-1][0] ] ].id > -1:
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

def unvisited_room_exists(direction, room, room_id, world):

    # print('here', direction, room, room_id)
    # prove the unknown room you check is not None
    if direction == 'n':
        if room['n'] == '?':
            north_neighbor = world.rooms[ room_id ].n_to
            return north_neighbor is not None
        # else:
            # if the room is already known 

    if direction == 's':
        if room['s'] == '?':
            south_neighbor = world.rooms[ room_id ].s_to
            return south_neighbor is not None
    
    if direction == 'e':
        if room['e'] == '?':
            east_neighbor = world.rooms[ room_id ].e_to
            # print('east neighbor', east_neighbor)
            return east_neighbor is not None

    if direction == 'w':
        if room['w'] == '?':
            west_neighbor = world.rooms[ room_id ].w_to
            # print('west neighbor', west_neighbor)

            return west_neighbor is not None
    return

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

def bfs(world, my_map, player_path_log):
    """
    Return a list containing the shortest path from
    starting_vertex to destination_vertex in
    breath-first order.
    """
    pass  # TODO
    # do bfs till we find a node with at least 1 unvisited edge
    # find a path for the player to traverse in dft

    # [print(i, self.vertices[i]) for i in self.vertices]
    visited = {i: 0 for i in world.rooms}

    my_queue = Queue()

    current_node = (opposite_directions[ player_path_log[-1][0] ], player_path_log[-1][1] )

    # start at the current room but oriented in the opposite direction
    my_queue.enqueue(current_node)

    # this is how we remember the path from the start vertex to the
    # destination(reversed linked list)
    parents = {i: [] for i in world.rooms}

    # find_unvisited_rooms(current_node, current_node[1], world)
    print(current_node)
    room = my_map[ current_node[1] ]
    print('backtracking')
    count = 0
    # we are traveling already visited rooms and ending when we have an unvisited room
    while len(find_unvisited_rooms(room, current_node[1], world)) == 0:#current_node != destination_vertex:

        # if count == 5:
        #     print('done backtracking')
        #     exit()
        print(count, len(find_unvisited_rooms(room, current_node[1], world)))
        print('current queue', my_queue.queue)
        current_node = my_queue.dequeue()
        # the queue runs out early
        print('current node', current_node)
        if current_node is None:
            break
        if visited[ current_node[1] ] == 0:
            print(current_node, 'just got visited')

            visited[ current_node[1] ] = 1
            # print('current node\'s directions')
            # print(my_map[ current_node[1] ])
            # directions are being stored insiscriminitatly
            # the unvisited edges should be looked at first
            # get a list of the edges to consider first
            # add them to the queue second

            # TODO:
            unvisited_rooms = []
            # need to know the room has at least 1 unvisited direction that exists
            for direction in my_map[ current_node[1] ]:
                # if my_map[ current_node[1] ][direction] is not None:
                    
                # The direction must point to a room that exists for it to be considered
                # if my_map[ current_node[1] ][direction] is not None:

                if my_map[ current_node[1] ][direction] == '?':
                    room = my_map[ current_node[1] ]

                    if unvisited_room_exists(direction, room, current_node[1], world):
                        unvisited_rooms.append((direction, my_map[ current_node[1] ][direction] ))
            if len(unvisited_rooms) > 0:
                print('we are done', current_node)
                print(unvisited_rooms)
                [print(i, parents[i]) for i in parents]
                # get the parent
                my_parent_id = my_map[  current_node[1]] [ opposite_directions[ current_node[0] ] ]
                print('the parent', opposite_directions[ current_node[0] ], my_parent_id)

                parents[ my_parent_id ] = [ current_node[0] , current_node[1] ]

                # , my_map[ current_node[1] ], direction)
                # if my_map[ current_node[1] ][direction] != '?':
                #     print('adding to parent array', [ direction, my_map[ current_node[1] ][direction] ], '->', current_node[1])
                #     parents[ current_node[1] ] = [ direction, my_map[ current_node[1] ][direction] ]
                break
                # current_node is the end node
                # exit()
            # there are no unvisited rooms adjacient to the current room
            else:
                for direction in my_map[ current_node[1] ]:

                    # the only rooms left are None, '?' that are dead ends and rooms that exist
                    if my_map[ current_node[1] ][direction] is not None:

                        # '?' that are dead end rooms
                        if my_map[ current_node[1] ][direction] == '?':
                            room = my_map[ current_node[1] ]
                            
                            # must be the part setting all the edges that don't go anywhere to None
                            # have to check this as we are visiting the same neighbors again
                            if not unvisited_room_exists(direction, room, current_node[1], world):
                                my_map[ current_node[1] ][direction] = None
                        # rooms that exist
                        else:
                            # don't add a room we have to do backwards to see
                            # if visited[ current_node[1] ] == 0:
                            # don't enque going backwards
                            if opposite_directions[ current_node[0] ] != direction:
                                print('adding new rooms, current node', current_node)
                                print(direction, my_map[ current_node[1] ][direction], visited[ current_node[1] ])

                                my_queue.enqueue((direction, my_map[ current_node[1] ][direction] ))

                                parents[ current_node[1] ] = [ direction, my_map[ current_node[1] ][direction] ]




            # if it does
                # our search is done
            # else
                # get it's neighbors and add them to the queue
            # the path is from the player's current room to the room with at least 1 unvisited direction

            # for direction in my_map[ current_node[1] ]:
            #     print(direction, my_map[ current_node[1] ][direction])
            #     # if my_map[ current_node[1] ][direction] is not None:
                    
            #     # The direction must point to a room that exists for it to be considered
            #     if my_map[ current_node[1] ][direction] is not None:
                    
            #         # unvisited room doesn't exist
            #         # eliminates the '?' that don't point to anywhere
            #         room = my_map[ current_node[1] ]

            #         # print('room direction', current_node[1], room, direction)
            #         # print(my_map[ room[direction] ], room[direction])
            #         # unvisited_room_exists(direction, room, room_id, world)
            #         # print(direction, 'direction id', my_map[ current_node[1] ][direction])
            #         # room is unvisited
            #         if my_map[ current_node[1] ][direction] == '?':

            #             # the unvisited room exists(these are not checked in the loop guard)
            #             # the assumption was to check the neighbors of the 
            #             # maybe adding nodes to the queue that aren't in the map yet is reaching too far
            #             print('unvisited node')
            #             if not unvisited_room_exists(direction, room, current_node[1], world):
            #                 my_map[ current_node[1] ][direction] = None
            #                 # print('got here')
            #                 # # (opposite_directions[ player_path_log[-1][0] ], player_path_log[-1][1] )
            #                 # # use my_map or world.rooms?
            #                 # # we are going to a known room or a n unknown room
            #                 # # get_room_directions(room, direction)
            #                 # room = world.rooms[ current_node[1] ]

            #                 # my_queue.enqueue((direction, get_room_directions(room, direction)  ))
                            
            #                 # # don't create a circular parent path
            #                 # if visited[ current_node[1] ] == 0:

            #                 #     parents[ current_node[1] ] = [ direction, world.rooms[ current_node[1] ][direction] ]

            #             # unvisited room exists
            #             else:
            #                 print('we have a new node to visit', direction)
            #                 print('player\'s current room', player_path_log[-1][1])
            #                 print('finishing backtracking')
            #                 print(parents)
            #                 [print(i, parents[i]) for i in parents]
            #                 tracker = [opposite_directions[ player_path_log[-1][0] ], player_path_log[-1][1] ]
            #                 print(tracker)
            #                 search_path = []
            #                 already_found = set()
            #                 while len(tracker) > 0 and tracker[1] > -1 and tracker[1] not in already_found:
            #                     search_path = [*search_path, tracker]
            #                     already_found.add(tracker[1])
            #                     tracker = parents[tracker[1]]
            #                     print(tracker)

            #                 # break
            #             #     my_map[ current_node[1] ][direction] = None
            #         # room has been visited
            #         elif my_map[ current_node[1] ][direction] > -1:
            #             print('got here')
            #             # get_room_directions(room, direction)
            #             # room = world.rooms[ current_node[1] ]
            #             # room[ get_room_directions(room, direction) ]
            #             # adding the room object instead of the room id
            #             my_queue.enqueue((direction, my_map[ current_node[1] ][direction] ))
            #             # print('visited', visited[ current_node[1] ])
            #             # don't create a circular parent path
            #             # if visited[ current_node[1] ] == 0:

            #             parents[ current_node[1] ] = [ direction, my_map[ current_node[1] ][direction] ]
            # print(count, len(find_unvisited_rooms(room, current_node[1], world)))


        count += 1
    print('ended backtracking')

    print(parents)
    [print(i, parents[i]) for i in parents]
    tracker = [opposite_directions[ player_path_log[-1][0] ], player_path_log[-1][1] ]
    print(tracker)
    search_path = []
    already_found = set()
    while len(tracker) > 0 and tracker[1] > -1 and tracker[1] not in already_found:
        search_path = [*search_path, tracker]
        already_found.add(tracker[1])
        tracker = parents[tracker[1]]
        print(tracker)
    # we already know what room they are in
    # print(search_path[1:])
    return search_path[1:]

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
    # TODO: this loop condition has been the only problem left but there is something else wrong with the large maze

    # while (current is not None) and (not(my_stack.size() == 0) or current['current'] in graph_for_stack):
    while len([visited[room] for room in visited if visited[room] == 0]) > 0:

        print('total traversal distance', len(player_path_log))
        if count == 70:
            print('loop has reached it\'s limit')
            return
        print('current count', count)
        print('player\'s current room', player.current_room.id)
        [print(i, my_map[i]) for i in my_map]

        # print(my_stack.stack)
        # print(my_stack.stack, current['current'], graph_for_stack[ current['current'] ], visited[ current['current'] ])
        # def travel(self, direction, show_rooms = False):

        # the current node is in the graph and it hasn't been visited
        # if current['current'] in graph_for_stack and visited[ current['current'] ] == 0:
        # print(dead_end( world.rooms[ player.current_room.id ], player_path_log))
        # print(visited[ player.current_room.id ])
        # room 0 has been visited already so we 
        if not dead_end( world.rooms[ player.current_room.id ], player_path_log):# and visited[ player.current_room.id ] == 0:

            # if visited[ player.current_room.id ] == 0:

            #     # We know there is at least 1 room unvisited
            #     visited[ player.current_room.id ] = 1

            # else:
                # we arent backtracking
                # the room has been visited before
                # we want to
            # print(player.current_room)

            # this is why each set of vertices needs to be an array
            # my_stack.push({'current': current['current'], 'ith_neighbor': 0})
            # current = { 'current': graph_for_stack[ current['current'] ][0],
            #             'ith_neighbor': 0}
            # should only choose unvisited rooms that exist
            # from a new rooms perspective the other room hasn't been added
            unvisited_rooms = find_unvisited_rooms( my_map[player.current_room.id],
                                                    player.current_room.id,
                                                    world)
            [print(i[0], i[1].id) for i in unvisited_rooms if i[1] is not None]
            # can we have a room with all visited nodes but you have to backtrack
            # from this point in time?
            # rooms is at interection of a loop but all it's neighbors have been visited
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
                if next_room_id not in my_map:
                    # print('next room to visit')
                    # print(next_direction, next_room_id)
                    # add next room to map
                    my_map[next_room_id] =  {'n': '?', 's': '?', 'w': '?', 'e': '?'}

                    # link current room to next room
                    my_map[player.current_room.id][next_direction] = next_room_id
                    # link next room to current room
                    my_map[next_room_id][ opposite_directions[next_direction] ] = player.current_room.id
                else:
                    my_map[player.current_room.id][next_direction] = next_room_id
                    # link next room to current room
                    my_map[next_room_id][ opposite_directions[next_direction] ] = player.current_room.id

                # TODO: set all remaining directions in my_map[player.current_room.id] that don't exist to None

                # move player to next room
                player.travel(next_direction)
                player_path_log.append((next_direction, next_room_id))
                # print('player\'s room', player.current_room.id)

                # [print(i, my_map[i]) for i in my_map]
                # [print(i) for i in player_path_log]
                # print()
                    # exit()
            else:
                print('our room has been already visited on all sides')
                # our room is already in the log so we get the penultimate node logged
                print(player_path_log[-2])
                # connect current room with prev
                # use bft to find the next
                exit()
            # else:
            #     # we cannot move forward any more
            #     # the player cannot go forward in any other direction so set all the rest of the unexplored paths to None
            #     for i in my_map[ player.current_room.id ]:
            #         if my_map[ player.current_room.id ][i] == '?':
            #             my_map[ player.current_room.id ][i] = None
            #     # bfs in the opposite direction
            #     print('bfs in the opposite direction')
        else:
            # we know the player is at a dead end so set all the rest of the unexplored paths to None
            for i in my_map[ player.current_room.id ]:
                if my_map[ player.current_room.id ][i] == '?':
                    my_map[ player.current_room.id ][i] = None
            
            # need to find the closest node with at least 1 room unexplored to restart dft

            # print('here')
            print(player.current_room)
            # current_room is a dead end

            [print(i, my_map[i]) for i in my_map]
            # do bfs in the opposite direction
            print('backtrack')
            # if last logged node has been logged before
            # seems to be working
            backtracking_path = bfs(world, my_map, player_path_log)
            print(backtracking_path)
            [print(i, my_map[i]) for i in my_map]
            
            for old_room in backtracking_path:
                player_path_log.append(tuple(old_room))
                player.travel(old_room[0])
                  
            print(player.current_room)
            [print(i) for i in player_path_log]
            print('finished backtracking')
            # correct for this round
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

# def dft_recursion():
    # if I reach the same node as already in the log then there is no need to backtrack
        # 121 node
    # loop pivot node where I can recurse back but not log the nodes
def dft_recursive_helper(   current_vertex,
                            visited,
                            rooms,
                            log,
                            looped_vertex,
                            my_new_exit,
                            count,
                            looped_verticies):


    # if current_vertex not in self.vertices:
    #     return
    log.append([my_new_exit + ' f', current_vertex])

    # print(count[0], current_vertex)
    # if count[0] == 358:
    #     return
    # if visited_verticies[current_vertex] == 1:
    #     print(current_vertex, 'has been seen before')
    #     looped_verticies.add(current_vertex)
    #     # if current_vertex == 121:
    #     #     exit()

    # print(rooms[current_vertex].get_exits())

    # if this is false we have an island away from the maze
    if len(rooms[current_vertex].get_exits()) > 0:
        # print('here')
        the_exits = rooms[current_vertex].get_exits()
        if len(the_exits) == 1:
            if visited[current_vertex] == 0:
                # print(current_vertex)
                visited[current_vertex] = 1

            # if the_exits[0] == opposite_directions[my_new_exit]:    
            #     # dead end
            #     print('dead end')
            # else:
            #     print('problem')
            #     exit()
        else:
            # node is not dead end and never been visited
            if visited[current_vertex] == 0:
                # print(current_vertex)
                visited[current_vertex] = 1

                count[0] += 1

                # going in every possible direction
                for my_exit in rooms[current_vertex].get_exits():

                    # log.append([my_new_exit, current_vertex])

                    next_room = rooms[current_vertex].get_room_in_direction(my_exit)
                    # print(my_exit )
                    if visited[next_room.id] == 0:
                        # if my_exit != opposite_directions[my_new_exit]:
                        # log direction and next node
                        dft_recursive_helper(   next_room.id,
                                                visited,
                                                rooms,
                                                log,
                                                looped_vertex,
                                                my_exit,
                                                count,
                                                looped_verticies)
                        log.append([opposite_directions[my_exit] + ' b', current_vertex])

                # back at the first time current_vertex was visited
                # if current_vertex in looped_vertex:
                #     print(f'we have found the first time {current_vertex} was visited')
                #     # chop off the log from end to current_vertex
                #     del looped_vertex[current_vertex]
                # else:
                #     print('about to recurse from call stack')
                    # log (reverse direction from last node on log, current node)
            # else:
            #     log.append(current_vertex)

            #     print('node has been visited before')
                # don't log this node
                # save value using the reference so it's avaliable to any call
                # looped_vertex[current_vertex] = 1
                # print('about to recurse from call stack')
                # log (reverse direction from last node on log, current node)

            

def dft_recursive(starting_vertex, rooms):
    """
    Print each vertex in depth-first order
    beginning from starting_vertex.

    This should be done using recursion.
    """
    pass  # TODO

    # I'm using a helper function so the graph doesn't need to fundamentally
    # altered for coloring verticies
    visited = {i: 0 for i in rooms}
    # starting_vertex is a number
    # log first node
    log = []
    count = [0]
    looped_verticies = set()
    dft_recursive_helper(starting_vertex, visited, rooms, log, {}, 'n', count, looped_verticies)
    # print('log count', len(log))
    # print(len([visited[i] for i in visited if visited[i] == 0]))
    all_items = {i: 1 for i in visited}
    for item in log:
        if item[1] in all_items:
            del all_items[item[1]]
    
    # print(len(all_items.keys()))

    log_visited = {i: 0 for i in rooms}
    # print(log)
    # print(len(log))
    return log
    # the next node appears to be right but the direction is wrong
    # direction = log[1][0]

    # tracker = log[0][1]
    # for i, item in enumerate(log[1:]):
    #     expected_room_id = item[1]

    #     tracker = rooms[tracker].get_room_in_direction(direction)
    #     if tracker == expected_room_id:
    #         if i + 1
    # print(looped_verticies)
# Load world
world = World()


# You may uncomment the smaller graphs for development and testing purposes.
# map_file = "maps/test_line.txt"
# map_file = "maps/test_cross.txt"
# map_file = "maps/test_loop.txt"
# map_file = "maps/test_loop_fork.txt"
map_file = "maps/main_maze.txt"

# Loads the map into a dictionary
room_graph=literal_eval(open(map_file, "r").read())
world.load_graph(room_graph)

# Print an ASCII map
world.print_rooms()

player = Player(world.starting_room)

# Fill this out with directions to walk
# traversal_path = ['n', 'n']
traversal_path = []
# exit()
print('player')
print(player)
print('done')
log = dft_recursive(0, world.rooms)
# dft(world, player)
traversal_path = [item[0].split(' ')[0] for item in log ]

# exit()
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


# correct map for test_loop.txt
# 0 {'n': 1, 's': 5, 'w': 7, 'e': 3} yes

# 1 {'n': 2, 's': 0, 'w': None, 'e': None} yes

# 2 {'n': None, 's': 1, 'w': None, 'e': None} yes

# 5 {'n': 0, 's': 6, 'w': None, 'e': None} yes

# 6 {'n': 5, 's': None, 'w': 11, 'e': None} yes

# 11 {'n': None, 's': None, 'w': 10, 'e': 6} yes

# 10 {'n': 9, 's': None, 'w': None, 'e': 11} yes

# 9 {'n': 8, 's': 10, 'w': None, 'e': None} yes

# 8 {'n': None, 's': 9, 'w': None, 'e': 7} yes

# 7 {'n': None, 's': None, 'w': 8, 'e': 0} yes

# 3 {'n': None, 's': None, 'w': 0, 'e': 4} yes
# 4 {'n': None, 's': None, 'w': 3, 'e': None}yes

# ('start', 0)
# ('n', 1)
# ('n', 2)
# ('s', 1)
# ('s', 0)
# ('s', 5)
# ('s', 6)
# ('w', 11)
# ('w', 10)
# ('n', 9)
# ('n', 8)
# ('e', 7)
# ('e', 0)
# ('e', 3)
# ('e', 4)
# ('w', 3)