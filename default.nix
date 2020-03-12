{ stdenv
, mkRosPackage
, robonomics_comm-nightly
, python3Packages
}:

mkRosPackage rec {
  name = "${pname}-${version}";
  pname = "liability_cacher";
  version = "0.2.0";

  src = ./.;

  propagatedBuildInputs = with python3Packages; [
    robonomics_comm-nightly
    pinatapy
    sqlalchemy
    psycopg2
    graphene
    flask
    flask-graphql
    flask-cors
    graphene-sqlalchemy
    graphene-sqlalchemy-filter
    ipfshttpclient
    sentry-sdk
  ];

  meta = with stdenv.lib; {
    description = "Liability Cacher";
    homepage = http://github.com/vourhey/liability_cacher;
    license = licenses.bsd3;
    maintainers = with maintainers; [ vourhey ];
  };
}
