import yaml,planning,random,math,sys,signal

trial_array = yaml.safe_load(open('output.yaml','r'))
input_config = yaml.safe_load(open('testing.yaml','r'))

#class TimedOutExc(Exception):
#	pass
#def deadline(timeout, *args):
#	def decorate(f):
#		def handler(signum,frame):
#			raise TimedOutExc()
#
#		def new_f(*args):
#			signal.signal(signal.SIGALRM, handler)
#			signal.alarm(timeout)
#			return f(*args)
#			signal.alarm(0)
#
#		new_f.__name__ = f.__name__
#		return new_f
#	return decorate


def signal_handler(sig, frame):
	print('You pressed Ctrl+C, exiting gracefully and saving data.')
	with open('output.yaml', 'w') as yaml_file:
		yaml.dump(trial_array, yaml_file, default_flow_style=False)
	sys.exit(0)

def run_random_trials(num_trials):
	for trial in range(num_trials):
		input_config['shots'] = []
		input_config['robots'] = []
		
		for _ in range(random.randrange(1,7)):
			robot_config = {}
			robot_config['max_speed'] = random.uniform(.1,40)
			robot_config['max_turn_rad'] = random.uniform(.1,20)
			robot_config['start_coord'] = [random.uniform(0,39.99),random.uniform(0,29.99),random.uniform(0,2*math.pi)]
			robot_config['fov'] = random.uniform(.1,180)
			robot_config['camera_orientation'] = random.uniform(0,360)
			robot_config['color'] = 'green'
	
			input_config['robots'].append(robot_config)
	
		for _ in range(random.randrange(1,7)):
			shot_config = {}
			shot_config['start_time'] = random.uniform(0,10)
			shot_config['end_time'] = random.uniform(shot_config['start_time'],10)
			min_uniform = min([robot['fov'] for robot in input_config['robots']])/2
			min_range = random.uniform(-min_uniform,min_uniform)
			max_range = random.uniform(min_range,min_uniform)
			shot_config['angle_range'] = [min_range,max_range]
			dist_range_min = random.uniform(0.1,10)
			dist_range_max = random.uniform(dist_range_min,10)
			shot_config['dist_range'] = [dist_range_min,dist_range_max]
			shot_config['actor_facing'] = random.uniform(0,360)
	
			input_config['shots'].append(shot_config)
		print(input_config,'robots: ',len(input_config['robots']),'shots: ',len(input_config['shots']))
		try:
			solved_config = planning.solve(input_config)
		except Exception as e:
			print('continuing')
			solved_config = -1
			pass

		trial_array.append(solved_config)
		with open('output.yaml', 'w') as yaml_file:
			yaml.dump(trial_array, yaml_file, default_flow_style=False)
	return trial_array

signal.signal(signal.SIGINT, signal_handler)
#input_config = {'shots': [{'start_time': 2.6643901911138945, 'end_time': 2.6807811587700265, 'angle_range': [10.686718871554048, 11.48533157993678], 'dist_range': [2.394279061710295, 8.427418963683962], 'actor_facing': 5.658097047233955}], 'actor': {'path': [[1, 15, 0, 5, 0], [39, 15, -90, 5, 10]], 'timestep_resolution': 0.1, 'color': 'red'}, 'animation': {'fps': 30, 'in_shot': 'red', 'out_shot': 'blue'}, 'map': {'path': 'white.png', 'height': 30, 'width': 40}, 'solve': {'resolution': 0.1, 'PRM_timeout': 20, 'PRM_point_dist': 10, 'PRM_degrees': 20, 'PRM_points': 'grid', 'PRM_num_points': 125}, 'robots': [{'max_speed': 36.5655006730657, 'max_turn_rad': 5.327102338461921, 'start_coord': [23.97257820588369, 17.217488717381322, 6.257437378036034], 'fov': 24.40262658825826, 'camera_orientation': 40.98403714310243, 'color': 'green'}, {'max_speed': 1.1357182998633917, 'max_turn_rad': 15.354308912946596, 'start_coord': [7.7389674748739425, 12.053580888794379, 5.840725081373186], 'fov': 100.26964802183124, 'camera_orientation': 117.62297896205577, 'color': 'green'}, {'max_speed': 1.411744069968982, 'max_turn_rad': 3.328787249772921, 'start_coord': [14.504051661052147, 26.96564288163752, 0.5031046933858032], 'fov': 114.98007689043996, 'camera_orientation': 151.87910205146096, 'color': 'green'}, {'max_speed': 24.43898155580719, 'max_turn_rad': 10.307200878125876, 'start_coord': [8.790078538334871, 12.379030036098268, 5.969583050836033], 'fov': 83.02011105200916, 'camera_orientation': 272.41578572712166, 'color': 'green'}]}
#solved_config = planning.solve(input_config,VERBOSE = True)
trial_array = yaml.safe_load(open('output.yaml','r'))
solved_config = run_random_trials(10000000000)
with open('output.yaml', 'w') as yaml_file:
	yaml.dump(solved_config, yaml_file, default_flow_style=False)
#if solved_config != -1:
#	planning.visualize(solved_config,saveName=None)