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

  propagatedBuildInputs = with python3Packages; [
    robonomics_comm
    pinatapy
    sqlalchemy
    psycopg2
    graphene
    flask
    flask-graphql
    flask-cors
    graphene-sqlalchemy
    ipfshttpclient
  ];

  meta = with stdenv.lib; {
    description = "Liability Cacher";
    homepage = http://github.com/vourhey/liability_cacher;
    license = licenses.bsd3;
    maintainers = with maintainers; [ vourhey ];
  };
}
