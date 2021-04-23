'use strict';

module.exports = {
    purge: {
        enabled: !!process.env.A32NX_PRODUCTION_BUILD,
        content: [
            './**/*.{jsx,tsx}',
        ],
    },
    darkMode: false, // or 'media' or 'class'
    theme: {
        extend: {
            width: () => ({
                'inr-tk': '13.45rem',
                'out-tk': '5.25rem'
            }),
            height: () => ({
                'efb': '50rem',
                'efb-nav': '45.75rem'
            }),
            margin: () => ({
                'ctr-tk-y': '15.425rem',
                'inn-tk-y': '11.25rem',
                'inn-tk-x': '19.525rem',
                'out-tk-y': '9.5rem',
                'out-tk-x': '38.25rem',
                'overlay-b-x': '27rem',
                'overlay-t-x': '30.5rem',
                'overlay-t-y': '14.75rem'
            }),
            rotate: () => ({
                '18.5': '18.5deg',
                '-18.5': '-18.5deg',
                '26.5': '26.5deg',
                '-26.5': '-26.5deg'
            })
        }
    },
    variants: { extend: {} },
    // eslint-disable-next-line global-require
    plugins: [require('@flybywiresim/tailwind-config')],
};
