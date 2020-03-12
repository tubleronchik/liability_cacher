{ rev    ? "3cb511d676c3ac4f5494d86b9ce655a74361faa8"             # The Git revision of nixpkgs to fetch
, sha256 ? "0h8imd7zg6yx63yn06wb5ka75mwd3x90g41w2n1zvahx9gsrrdk7" # The SHA256 of the downloaded data
}:

builtins.fetchTarball {
  url = "https://github.com/airalab/airapkgs/archive/${rev}.tar.gz";
  inherit sha256;
}
