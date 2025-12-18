const webpack = require('webpack');
const dotenv = require('dotenv');
const path = require('path');

module.exports = function (context, options) {
  const env = dotenv.config({ path: path.resolve(context.siteDir, '.env') }).parsed;

  const envKeys = Object.keys(env).reduce((prev, next) => {
    prev[`process.env.${next}`] = JSON.stringify(env[next]);
    return prev;
  }, {});

  return {
    name: 'inject-env-plugin',
    configureWebpack(config, isServer) {
      return {
        plugins: [new webpack.DefinePlugin(envKeys)],
      };
    },
  };
};
