{ stdenv
, mkRosPackage
, robonomics_comm
, python3Packages
}:

mkRosPackage rec {
  name = "${pname}-${version}";
  pname = "liability_cacher";
  version = "0.1.0";

  src = ./.;

  propagatedBuildInputs = [
    robonomics_comm
    python3Packages.pinatapy
    python3Packages.sqlalchemy
    python3Packages.psycopg2
    python3Packages.graphene
    python3Packages.flask
  ];

  meta = with stdenv.lib; {
    description = "Liability Cacher";
    homepage = http://github.com/vourhey/liability_cacher;
    license = licenses.bsd3;
    maintainers = with maintainers; [ vourhey ];
  };
}
