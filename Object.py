class Object(object):

	def __init__(self,object_id , object_name , object_type):
		self.__object_id  = object_id
		self.__object_name = object_name
		self.__object_type = object_type
	
	def getObjectId(self):
		return self.__object_id
	
	def getObjectname(self):
		return self.__object_name

	def getObjectType(self):
		return self.__object_type



		
