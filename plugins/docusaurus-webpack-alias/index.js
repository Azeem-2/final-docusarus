const path = require('path');

/**
 * Docusaurus Webpack Alias Plugin
 * 
 * Configures webpack to resolve @/ imports to the src/ directory
 */
module.exports = function webpackAliasPlugin(context, options) {
  return {
    name: 'docusaurus-webpack-alias',
    configureWebpack(config, isServer) {
      const existingResolve = config.resolve || {};
      const existingAlias = existingResolve.alias || {};
      
      return {
        resolve: {
          ...existingResolve,
          alias: {
            ...existingAlias,
            '@': path.resolve(context.siteDir, 'src'),
          },
        },
      };
    },
  };
};

