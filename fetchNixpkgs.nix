{ rev    ? "e1563366b83d6bb159b370dd5c041a28819b17d1"             # The Git revision of nixpkgs to fetch
, sha256 ? "169njmml30d2lb15921d22s27rg22ck7bbcvj36ralzgmdzc3f6l" # The SHA256 of the downloaded data
}:

builtins.fetchTarball {
  url = "https://github.com/airalab/airapkgs/archive/${rev}.tar.gz";
  inherit sha256;
}
