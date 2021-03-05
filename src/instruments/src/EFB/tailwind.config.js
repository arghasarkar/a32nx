'use strict';

module.exports = {
    purge: {
        enabled: !!process.env.A32NX_PRODUCTION_BUILD,
        content: [
            './**/*.{jsx,tsx}',
        ],
    },
    darkMode: false, // or 'media' or 'class'
    theme: { extend: { height: () => ({ efb: '50rem' }) } },
    variants: { extend: {} },
    // eslint-disable-next-line global-require
    plugins: [require('@flybywiresim/tailwind-config')],
};
