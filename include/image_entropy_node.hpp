
const int width = 160;
const int height = 120;
const int interval = 10;
const int num_events = 255 / interval + 1;
const int queue_size = 1;

void initialize_occs(int occs[width][height][num_events]);
double calculate_entropy(int occs[width][height][num_events], int x, int y);
int sum_row(int occs[width][height][num_events], int x, int y);
geometry_msgs::Point32 transform_pixel(int px, int py,
        geometry_msgs::PoseStamped pose);
void reset_occs(int occs[width][height][num_events], int x, int y);
double radians(double degree);
void pose_callback(geometry_msgs::PoseStamped);
