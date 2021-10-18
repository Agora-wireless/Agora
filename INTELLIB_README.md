## Intel libraries installation
  * Intel MKL and Intel compiler can be installed from [Intel oneAPI Toolkits](https://www.intel.com/content/www/us/en/developer/tools/oneapi/toolkits.html#gs.dg5bw3).
  Intel MKL is available in [oneAPI base Toolkit](https://www.intel.com/content/www/us/en/developer/tools/oneapi/base-toolkit-download.html?operatingsystem=linux&distributions=webdownload&options=online). Intel compiler is available in [oneAPI HPC Toolkit](https://www.intel.com/content/www/us/en/developer/tools/oneapi/hpc-toolkit-download.html?operatingsystem=linux&distributions=webdownload&options=online).
  * For each Toolkit, follow the instructions for `Command Line Download` to install the online installer. 
  	* For base Toolkit, run
  	  <pre>
  	  $ wget https://registrationcenter-download.intel.com/akdlm/irc_nas/18236/l_BaseKit_p_2021.4.0.3422.sh
	  $ sudo bash l_BaseKit_p_2021.4.0.3422.sh	
  	  </pre>
  	  After running the GUI, select `Modify` to customize the libraries. 
  	  `Math Kernel Library` and `Integrated Performance Primitives` are required for Agora. The others are optional.
  	* For HPC Toolkit, run
  	  <pre>
  	  $ wget https://registrationcenter-download.intel.com/akdlm/irc_nas/18211/l_HPCKit_p_2021.4.0.3347.sh
	  $ sudo bash l_HPCKit_p_2021.4.0.3347.sh	
  	  </pre>
  	  After running the GUI, select `Modify` to customize the libraries. 
  	  `DPC++/C++ Compiler` and `C++ Compiler Classic` are required for Agora. The others are optional.
