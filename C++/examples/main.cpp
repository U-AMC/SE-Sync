#include "SESync/SESync.h"
#include "SESync/SESync_utils.h"
#include <unistd.h>
#include <fstream>

#ifdef GPERFTOOLS
#include <gperftools/profiler.h>
#endif

using namespace std;
using namespace SESync;

bool write_poses = true;
unsigned int microsecond = 1000000;

int main(int argc, char **argv) {
  if (argc != 2) {
    cout << "Usage: " << argv[0] << " [input .g2o file]" << endl;
    exit(1);
  }

  size_t num_poses;
  //g2o iostream uses custom setup inside SE-sync
  measurements_t measurements = read_g2o_file(argv[1], num_poses);
  cout << "Loaded " << measurements.size() << " measurements between "
       << num_poses << " poses from file " << argv[1] << endl
       << endl;
  if (measurements.size() == 0) {
    cout << "Error: No measurements were read!"
         << " Are you sure the file exists?" << endl;
    exit(1);
  }

//================================//
  //SEsync option setup from here
  SESyncOpts opts;
  opts.verbose = false; // Print output to stdout

  // Initialization method
  // Options are:  Chordal, Random
  opts.initialization = Initialization::Chordal;

  // Specific form of the synchronization problem to solve
  // Options are: Simplified, Explicit, SOSync
  opts.formulation = Formulation::Simplified;

  // Initial
  opts.num_threads = 16;

#ifdef GPERFTOOLS
  ProfilerStart("SE-Sync.prof");
#endif
//=================================//

measurements_t measurements_temp;
SESyncResult results;

for(int cnt = 0; cnt < num_poses; ++cnt){
  measurements_temp.push_back(measurements.at(cnt));
  //g2o measurement vector saved every 10 node (vecot+=10)
  if(cnt%10==0){
    //check measurement vector size by edge index
    cout << measurements_temp.front() << endl; 
    cout <<measurements_temp.back() << endl;
    // usleep(5 * microsecond); //wait
    // #ifdef GPERFTOOLS
//   ProfilerStop();
// #endif 

  /// RUN SE-SYNC!
    results = SESync::SESync(measurements_temp, opts);

      for(int iter = 0; iter != cnt ; ++iter){
        Eigen::Vector3d opt_mat; 
        //vector output check
        // cout << measurements.at(cnt) << endl;
        // raw_poses_file << measurements.at(cnt); //raw
        
        //make it row
        opt_mat.block(0, 0, 1, 2) = results.xhat.block(0,iter,2,1);
        cout << opt_mat.block(0, 0, 1, 2) << endl;
        // opt_poses_file << opt_mat.block(0,0,1,2) << endl; //t
        // results.xhat.block(0, n + 2*cnt, 2, 2); //R
        }
      // opt_poses_file << results.xhat;
      // raw_poses_file.close();
      // opt_poses_file.close();
      usleep(5 * microsecond);
    }

  
// #ifdef GPERFTOOLS
//   ProfilerStop();
// #endif

    // if (cnt%10 == 0 || num_poses == cnt) {
    //   /// RUN SE-SYNC every 50 poses!
    //   results = SESync::SESync(measurements_temp, opts);
    //   Eigen::Vector3d opt_mat; 
    //   // Write output
    //   string count;
    //   count = to_string(cnt);
    //   string form = ".txt";
    //   string raw_pose = "raw_poses" + form;
    //   string opt_pose = "opt_poses_";
    //   string fname_opt = opt_pose + count + form;
      
    //   // cout << "Saving final poses to raw_pose + opt_pose: " << raw_pose << "," << opt_pose << endl;
      
    //   ofstream raw_poses_file(raw_pose);
    //   ofstream opt_poses_file(fname_opt);
      
    //   for(int cnt = 0; cnt != measurements.size() ; ++cnt){
    //     //vector output check
    //     // cout << measurements.at(cnt) << endl;
    //     raw_poses_file << measurements.at(cnt); //raw
        
    //     //make it row
    //     opt_mat.block(0, 0, 1, 2) = results.xhat.block(0,cnt,2,1);
    //     // cout << opt_mat.block(0, 0, 1, 2) << endl;
    //     opt_poses_file << opt_mat.block(0,0,1,2) << endl; //t
    //     //results.xhat.block(0, n + 3*cnt, 3, 3); //R
    //     }
    //   // opt_poses_file << results.xhat;
    //   raw_poses_file.close();
    //   opt_poses_file.close();
    //   usleep(5 * microsecond);
    // }
  usleep(1 * microsecond);
  }

}
