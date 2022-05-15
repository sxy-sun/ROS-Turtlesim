class PID:
	"""
	Discrete PID control
	"""

	def __init__(self, P=2, D=1, Derivator=0):

		self.Kp=P
		self.Kd=D
		self.Derivator=Derivator
		self.error=0.0

	def update_vel(self, error):
		"""
		Calculate PID output value for given reference input and feedback
		"""

		self.error = error

		self.P_value = self.Kp * self.error
		self.D_value = self.Kd * ( self.error - self.Derivator)
		self.Derivator = self.error


		PID = self.P_value + self.D_value

		return PID