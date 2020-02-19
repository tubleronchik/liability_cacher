{ rev    ? "e09e47483d135ef83a828d55f256c4c9dc9e774c"             # The Git revision of nixpkgs to fetch
, sha256 ? "02dl68vw2hyg9i57yyyg553gylklgmw5h945i606c0h97qxz0a7m" # The SHA256 of the downloaded data
}:

builtins.fetchTarball {
  url = "https://github.com/airalab/airapkgs/archive/${rev}.tar.gz";
  inherit sha256;
}
