/**
 * WebsocketImageComponent is a custom HTML element that displays an image received from a websocket server.
 * It also includes a button to toggle sending of te images.
 */
class WebsocketImageComponent extends HTMLElement {
    /**
     * Constructor of the WebsocketImageComponent.
     * Calls the render method to create the component's UI and establishes a websocket connection.
     */
    constructor() {
        super();

        this.render();
        this.connectWS();
    }

    /**
     * Render the component by creating an image tag and a button.
     * The button is used to toggle the websocket connection.
     */
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

        this.appendChild(header);
        this.appendChild(button);
        this.appendChild(this.img);
    }

    /**
     * Send a 'toggle' message to the server to toggle the websocket connection.
     */
    toggleWS() {
        // send a toggle message to the server
        this.ws.send('toggle');
    }

    /**
     * Establish a new websocket connection.
     * When a new message is received, update the image.
     * The image is sent as a base64 encoded JPEG.
     */
    connectWS() {
        // create a new websocket connection
        this.ws = new WebSocket(`ws://localhost:8000/ws/${this.getAttribute('id')}`);
        this.ws.onerror = (event) => {
            console.error('Websocket error:', event);
        }
        this.ws.binaryType = 'arraybuffer';
        // when a new message is received, update the image. the image is send as base64 encoded jpeg
        this.ws.onmessage = (event) => {
            const img_src = "data:image/jpeg;base64," + event.data;
            // check if it is the same as the current image
            if (this.img.src === img_src) {
                return console.log("same img")
            }
            this.img.src = img_src;
        }
    }
}

// define the custom element
customElements.define('websocket-image', WebsocketImageComponent);
