//#include "SEE.h"
//
//void print_view(SeeView& view) {
//	for (int i = 0; i < 3; i++)
//		cout << view.data[i] << " ";
//	for (int i = 0; i < 3; i++)
//		cout << view.view[i] << " ";
//	std::cout << "\n";
//}
//
//char* extract_filename(int argc, char* argv[]) {
//	if (argc < 2) {
//		printf("No filename provided.\n");
//		printf("Usage: %s <filename>\n", argv[0]);
//		return NULL;
//	}
//
//	return argv[1];
//}
//
//int main(int argc, char* argv[])
//{
//	char* filename = extract_filename(argc, argv);
//	if (filename != NULL) {
//		printf("The filename provided is: %s\n", filename);
//	}
//	else {
//		return -2;
//	}
//
//	SEE see(filename);
//
//	SeePointCloudPtr cloud(new pcl::PointCloud<SeePoint>);
//
//	if (pcl::io::loadPCDFile<SeePoint>("test.pcd", *cloud) == -1)
//	{
//		PCL_ERROR("Couldn't read file your_point_cloud.pcd\n");
//		return -1;
//	}
//
//	SeeView view, nbv;
//	view.x = -1.25516047;
//	view.y = 0.16405614;
//	view.z = 0.90033961;
//	view.view_x = 0.81845733;
//	view.view_y = -0.11847473;
//	view.view_z = -0.56222001;
//
//	see.SearchNBVOnce(cloud, view, nbv);
//
//	print_view(nbv);
//
//	return 0;
//}