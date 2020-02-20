{ rev    ? "31851e0d26001ee8d1e6fe0563d7c4a27887aebd"             # The Git revision of nixpkgs to fetch
, sha256 ? "0mzndfa3w875mhli7mzr34994rrdra4yzz29x5iqmy6sfm819zgm" # The SHA256 of the downloaded data
}:

builtins.fetchTarball {
  url = "https://github.com/airalab/airapkgs/archive/${rev}.tar.gz";
  inherit sha256;
}
