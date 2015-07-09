import random


def rand_num_as_str(n):
	rand_id = ""
	rand_id = rand_id.join(["%s" % random.randint(0,9) for num in range(0, n)]);
	return rand_id
