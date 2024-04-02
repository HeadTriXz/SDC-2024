class WebsocketImageComponent extends HTMLElement {

    constructor() {
        super();


        this.render();
        this.connectWS();
    }

    render() {
        // create an image tag
        const header = document.createElement('h4')
        header.textContent = this.getAttribute('id');


        this.img = document.createElement('img');
        this.img.style.width = '100%';
        this.img.style.height = 'auto';
        const button = document.createElement('button');
        button.textContent = 'Toggle';
        button.onclick = () => this.toggleWS();

        // append the image and button to the dom
        this.appendChild(header);
        this.appendChild(button);
        this.appendChild(this.img);



    }

    toggleWS() {
        // send a toggle message to the server
        console.log('toggle')
        this.ws.send('toggle');
    }

    connectWS() {
        // create a new websocket connection
        this.ws = new WebSocket(`ws://localhost:8000/ws/${this.getAttribute('id')}`);
        this.ws.onerror = (event) => {
            console.error('Websocket error:', event);
        }
        this.ws.binaryType = 'arraybuffer';

        // when a new message is received, update the image. the image is send as base64 encoded jpeg
        this.ws.onmessage = (event) => {
            console.log('data')
            const img_src = "data:image/jpeg;base64," + event.data;
            // check if it is the same as the current image
            if (this.img.src == img_src) {
                return console.log("same img")
            }
            this.img.src = img_src;
        }
    }
}

// define the custom element
customElements.define('websocket-image', WebsocketImageComponent);