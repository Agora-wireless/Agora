#include <armadillo>

int main () {
  arma::arma_rng::set_seed_random();

  arma::cx_fmat mat_tmp(5, 1, arma::fill::randu);     // generate a random complex matrix   mat_tmp

  std::cout << mat_tmp << std::endl;                  // print out mat_tmp

  // mat_tmp /= arma::square(arma::conv_to<arma::cx_fmat>::from(arma::abs(mat_tmp)));    // sq-normalizing mat_tmp

  mat_tmp /= arma::conv_to<arma::cx_fmat>::from(arma::square((arma::abs(mat_tmp))));


  std::cout << mat_tmp << std::endl;                                  // print out normalized mat_tmp

  std::cout << arma::abs(mat_tmp) << std::endl;           // check magnitude of each element of mat_tmp 

  // another randomly generated vec, from which a vec orthogonal
  // to mat_tmp is produced by the Gram-schmidt procedure
  arma::cx_fmat mat_tmp_2(5, 1, arma::fill::randu);
  // arma::cx_fmat mat_tmp_ort = mat_tmp_2 - ((mat_tmp_2.t() * mat_tmp) / (mat_tmp.t() * mat_tmp) ) * mat_tmp;



  std::cout << mat_tmp_2.t() * mat_tmp << std::endl;
  std::cout << arma::norm(mat_tmp,2) << std::endl;
  std::cout << mat_tmp.t() * mat_tmp << std::endl;
  // std::cout << mat_tmp_ort * mat_tmp << std::endl;
  std::cout << ((mat_tmp_2.t() * mat_tmp) / (mat_tmp.t() * mat_tmp) ) << std::endl;


  float scale = 5;

  scale = scale * scale;

  std::cout << scale << std::endl;

/***********************************************************
*      Beamweight adjustment method#2
1. Each SC is associated with an ASM precoder (i.e., a beamweight vector
), where some elements are zeroed out.

2. Do global power scaling first, for each SC's precoder, to avoid clipping.  
Operationally, divide each weight vector by its maximum. Call the outcome 
the "globally-scaled precoder".

3. Calculate the gain of effective channel under each precoder, which is the 
MAGNITUDE of inner product between DL CSI and globally scaled precoder. Call the 
outcome the beta_1, beta_2, beta_3.

4. Do the second beamweight adjustment to avoid effective channel's amplitude 
mismatch: 
  a). find the minimum effective channel gain; call it beta
  b). scale each "globally-scaled precoder" one-by-one; E.g., element-wise
  multiplying precoder_i with (beta/beta_i).
***********************************************************/
  arma::mat h;
  h.set_size(3, 1);
  h(0,0) = 0.1;
  h(1,0) = 0.5;
  h(2,0) = 0.7;
  std::cout << "this is original channel vector h:" << h << std::endl;   //checked

  arma::mat w1; arma::mat w2; arma::mat w3;
  w1.set_size(3,1); w2.set_size(3,1); w3.set_size(3,1);
  w1(0,0) = 0.1;
  w1(1,0) = 0.5;
  w1(2,0) = 0.0;
  w2(0,0) = 0.0;
  w2(1,0) = 0.5;
  w2(2,0) = 0.7;
  w3(0,0) = 0.1;
  w3(1,0) = 0.0;
  w3(2,0) = 0.7;
  w1*= 1/arma::max(w1);   // global power scaling to avoid clipping
  w2*= 1/arma::max(w2);
  w3*= 1/arma::max(w3);
  std::cout << "this is w1 right after global power scaling:" << w1 << std::endl;
  std::cout << "this is w2 right after global power scaling:" << w2 << std::endl;
  std::cout << "this is w3 right after global power scaling:" << w3 << std::endl;   //checked

  arma::mat heff_1; arma::mat heff_2; arma::mat heff_3; 
  heff_1 = arma::abs(h.t() * w1);     // calculating the gain of the MISO effective channel
  heff_2 = arma::abs(h.t() * w2);
  heff_3 = arma::abs(h.t() * w3);
  std::cout << "this is heff_1 before second scaling:" << heff_1 << std::endl;
  std::cout << "this is heff_2 before second scaling:" << heff_2 << std::endl;
  std::cout << "this is heff_3 before second scaling:" << heff_3 << std::endl;     //checked

  arma::mat heff_vec; heff_vec.set_size(3, 1);
  heff_vec(0,0) = arma::conv_to<float>::from(heff_1);
  heff_vec(1,0) = arma::conv_to<float>::from(heff_2);
  heff_vec(2,0) = arma::conv_to<float>::from(heff_3);
  w1 *= arma::min(heff_vec)/arma::conv_to<float>::from(heff_1);
  w2 *= arma::min(heff_vec)/arma::conv_to<float>::from(heff_2);
  w3 *= arma::min(heff_vec)/arma::conv_to<float>::from(heff_3);
  std::cout << "this is w1 after second scaling:" << w1 << std::endl;
  std::cout << "this is w2 after second scaling:" << w2 << std::endl;
  std::cout << "this is w3 after second scaling:" << w3 << std::endl;      //checked

  heff_1 = arma::abs(h.t() * w1);    // calculate effective channel after beamweight adjustment 
  heff_2 = arma::abs(h.t() * w2);
  heff_3 = arma::abs(h.t() * w3);
  std::cout << "this is heff_1 before second scaling:" << heff_1 << std::endl;
  std::cout << "this is heff_2 before second scaling:" << heff_2 << std::endl;
  std::cout << "this is heff_3 before second scaling:" << heff_3 << std::endl;     // checked


  arma::mat calib = arma::randn(1,8);
  std::cout << "this is calib of 1-by-8:" << calib << std::endl;
  arma::mat matcsi = arma::randn(9,1);
  std::cout << "this is calib of 9-by-1:" << matcsi << std::endl;
  arma::mat diag_calib = arma::diagmat(calib);
  std::cout << "this is calib after diagonal transform (call it diag_calib): \n" << diag_calib << std::endl;
  arma::mat inv_diag_calib = arma::inv(diag_calib);
  std::cout << "this is inversed diag_calib: \n" << inv_diag_calib << std::endl;
  arma::mat result = inv_diag_calib * matcsi;
  std::cout << "this is final result: \n" << result << std::endl;

}