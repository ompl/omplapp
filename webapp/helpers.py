import random

class LogOutputHandler(object):
	"""
	Object for handling various levels of logging.
	Logging levels:
		0 = Silent (probablly shouldn't use this)
		1 = Errors only
		2 = Level 0 and warnings
		3 = Levels 1, 2 and debugging
		4 = Levels 1, 2, 3 and info
	"""

	def __init__(self, log_level):
		# Specifies the level of logging should be printed to the console
		self.log_level = log_level

		self.messages = "Messages: \n"

	def error(self, text):
		if self.log_level > 0:
			print("# Error:    " + str(text))

	def warn(self, text):
		if self.log_level > 1:
			print("# Warning:    " + str(text))

	def debug(self, text):
		if self.log_level > 2:
			print("# Debug:    " + str(text))

	def info(self, text):
		# Store the message
		self.messages += str(text)

		if self.log_level > 3:
			print ("# Info:    " + str(text))

	def getMessages(self):
		# Info messages are stored and can be retrieved via this function to
		# send to the client
		return self.messages

	def clearMessages(self):
		# Clear the stored messages, this should be called after sending to client
		self.messages = "Messages: \n"


def rand_num_as_str(n):
	rand_id = ""
	rand_id = rand_id.join(["%s" % random.randint(0,9) for num in range(0, n)]);
	return rand_id
