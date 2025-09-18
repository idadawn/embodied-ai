/** @type {import('next').NextConfig} */
const nextConfig = {
  experimental: {
    typedRoutes: true,
    serverComponentsExternalPackages: ['@rerun-io/web-viewer']
  },
  webpack: (config) => {
    config.experiments = {
      ...(config.experiments ?? {}),
      asyncWebAssembly: true
    };

    config.module.rules.push({
      test: /\.wasm$/,
      type: 'webassembly/async'
    });

    return config;
  }
};

export default nextConfig;
