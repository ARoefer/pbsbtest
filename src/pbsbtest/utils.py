import rospkg

def res_pkg_path(rpath, resolver=rospkg.RosPack()):
	if rpath[:10] == 'package://':
		rpath = rpath[10:]
		pkg = rpath[:rpath.find('/')]
		return resolver.get_path(pkg) + rpath[len(pkg):]
	return rpath