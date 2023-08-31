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

int main(int argc, char **argv) {
  if (argc != 2) {
    cout << "Usage: " << argv[0] << " [input .g2o file]" << endl;
    exit(1);
  }

  size_t num_poses;
  measurements_t measurements = read_g2o_file(argv[1], num_poses);
  cout << "Loaded " << measurements.size() << " measurements between "
       << num_poses << " poses from file " << argv[1] << endl
       << endl;
  if (measurements.size() == 0) {
    cout << "Error: No measurements were read!"
         << " Are you sure the file exists?" << endl;
    exit(1);
  }

  SESyncOpts opts;
  opts.verbose = true; // Print output to stdout

  // Initialization method
  // Options are:  Chordal, Random
  opts.initialization = Initialization::Chordal;

//   opts.preconditioner = Preconditioner::RegularizedCholesky;

  // Specific form of the synchronization problem to solve
  // Options are: Simplified, Explicit, SOSync
  opts.formulation = Formulation::Simplified;

  // Initial
  opts.num_threads = 4;

#ifdef GPERFTOOLS
  ProfilerStart("SE-Sync.prof");
#endif

  /// RUN SE-SYNC!
  SESyncResult results = SESync::SESync(measurements, opts);

#ifdef GPERFTOOLS
  ProfilerStop();
#endif

  if (write_poses) {
    // Write output for raw measurement
    string fname_raw = "raw_t.txt";
    string fname_opt = "opt_poses.txt";
    cout << "Saving raw poses to file: " << fname_raw << endl;
    ofstream poses_raw(fname_raw), poses_opt(fname_opt);

    for(int cnt=0;cnt!=measurements.size();cnt++){
        poses_raw<<measurements.at(cnt).t.transpose() << endl;
    }

    for(int iter = 0; iter != measurements.size() ; ++iter){
        Eigen::RowVector2d opt_t;
        Eigen::Matrix2d opt_R;

        //xhat, make it row_vector
        // cout << results.xhat.block(0, iter, 2, 1) << endl;
        opt_t = -1* results.xhat.block(0,iter,2,1).transpose();
        opt_R.block(0, 0, 2, 2) = results.xhat.block(0, 2 + 3*iter,2,2);
        // cout << opt_t.block(0, 0, 2, 1) << endl;
        
        poses_opt << opt_t << endl; //t
        // results.xhat.block(0, n + 2*cnt, 2, 2); //R
        }
      // opt_poses_file << results.xhat;
        poses_raw.close(); poses_opt.close();
      // meas_recursive = 
    
  }
}
